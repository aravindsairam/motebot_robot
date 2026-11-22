"""The assistant's brain: a conversation with a language model that may look things up on the
robot (its tools) before it answers. No ROS imports: the robot is reached through `run_tool`.

One question is one call of ask(). The model answers, or asks for tools; their results go back
to it, and so on, up to MAX_ROUNDS times. Only a question that was answered is kept in the
conversation, so a failure (no internet, a refusal) leaves the conversation as it was.
"""

import base64
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import anthropic

from .tools import ToolOutput

MODEL = "claude-opus-5-5"
MAX_TOKENS = 4000  # answers are a few sentences; this leaves room for the model's own reasoning
MAX_ROUNDS = 8  # model calls for one question: tools are looked up between them
MAX_EXCHANGES = 40  # questions kept in one conversation; after that it starts afresh
FALLBACK_BETA = "server-side-fallback-2026-07-01"


@dataclass
class Answer:
    ok: bool  # False: no answer could be had; `text` says why, in plain words
    text: str
    tools_used: List[str] = field(default_factory=list)
    input_tokens: int = 0
    output_tokens: int = 0
    picture_jpeg: bytes = b""  # the last picture a tool took, shown with the answer
    mark: Optional[Tuple[float, float]] = None  # the last place on the map a tool named


@dataclass
class Conversation:
    messages: list = field(default_factory=list)
    exchanges: int = 0
    last_used: float = field(default_factory=time.monotonic)


class Brain:
    def __init__(self, client, system: str, tools: list, run_tool: Callable[[str, dict], ToolOutput],
                 model: str = MODEL, effort: str = "low", fallback: bool = True, max_rounds: int = MAX_ROUNDS, log=print,
                 on_tool: Callable[[str], None] = lambda _name: None):
        """`run_tool(name, input)` does what the model asked; `on_tool(name)` is told first."""
        self.client = client
        self.system = [{"type": "text", "text": system, "cache_control": {"type": "ephemeral"}}]
        self.tools = tools
        self.run_tool = run_tool
        self.model = model
        self.effort = effort
        self.fallback = fallback
        self.max_rounds = max_rounds
        self.log = log
        self.on_tool = on_tool

    def _create(self, messages):
        options = dict(model=self.model, max_tokens=MAX_TOKENS, system=self.system, tools=self.tools, messages=messages,
                       output_config={"effort": self.effort})
        if self.fallback:
            # If the model declines a request, the service answers it with another model instead.
            return self.client.beta.messages.create(betas=[FALLBACK_BETA], extra_body={"fallbacks": "default"}, **options)
        return self.client.messages.create(**options)

    def ask(self, conversation: Conversation, text: str) -> Answer:
        if conversation.exchanges >= MAX_EXCHANGES:
            conversation.messages, conversation.exchanges = [], 0
        conversation.last_used = time.monotonic()
        messages = list(conversation.messages) + [{"role": "user", "content": text}]
        used: List[str] = []
        tokens_in = tokens_out = 0
        picture, mark = b"", None
        for _ in range(self.max_rounds):
            try:
                response = self._create(messages)
            except anthropic.AuthenticationError:
                return Answer(False, "My API key was not accepted, so I cannot think right now. Check the key on the robot.", used)
            except anthropic.PermissionDeniedError:
                return Answer(False, "My API key is not allowed to use this model. Check the key's account.", used)
            except anthropic.RateLimitError:
                return Answer(False, "Too many questions at once for my account. Try again in a minute.", used)
            except anthropic.APIConnectionError:
                return Answer(False, "I cannot reach the internet, so I cannot think right now. The robot still works from the app.", used)
            except anthropic.APIStatusError as exc:
                self.log(f"the language model answered with an error: {exc.status_code} {exc.message}")
                return Answer(False, f"Something went wrong on the model's side ({exc.status_code}). Try again.", used)
            except Exception as exc:  # anything else must still give the asker an answer
                self.log(f"asking the language model failed: {exc!r}")
                return Answer(False, "Something went wrong while I was thinking. Try again.", used)
            usage = getattr(response, "usage", None)
            tokens_in += getattr(usage, "input_tokens", 0) or 0
            tokens_out += getattr(usage, "output_tokens", 0) or 0
            messages.append({"role": "assistant", "content": response.content})
            if response.stop_reason == "tool_use":
                results = []
                for block in response.content:
                    if getattr(block, "type", "") != "tool_use":
                        continue
                    used.append(block.name)
                    try:
                        self.on_tool(block.name)
                        out = self.run_tool(block.name, dict(block.input or {}))
                    except Exception as exc:  # a tool must never take the conversation down
                        self.log(f"tool {block.name} failed: {exc!r}")
                        out = ToolOutput(f"that did not work: {exc}", True)
                    content = [{"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                                            "data": base64.standard_b64encode(p).decode("ascii")}}
                               for p in out.pictures]
                    content.append({"type": "text", "text": out.text})
                    result = {"type": "tool_result", "tool_use_id": block.id, "content": content}
                    if out.is_error:
                        result["is_error"] = True
                    results.append(result)
                    if out.pictures:
                        picture = out.pictures[-1]
                    if out.mark is not None:
                        mark = out.mark
                messages.append({"role": "user", "content": results})  # every result of the turn in one message
                continue
            reply = " ".join(b.text.strip() for b in response.content if getattr(b, "type", "") == "text" and b.text.strip())
            if response.stop_reason == "refusal":
                return Answer(False, reply or "I can't help with that one.", used, tokens_in, tokens_out)
            if response.stop_reason == "max_tokens":
                if any(getattr(b, "type", "") == "tool_use" for b in response.content):  # cut off inside a tool call
                    return Answer(False, "That answer got too long for me. Could you ask a narrower question?", used, tokens_in, tokens_out)
                reply = (reply + " ..." if reply else "That answer got too long for me. Could you ask a narrower question?")
            if not reply:  # an empty turn is not kept: sent back, the API would refuse every later question
                return Answer(False, "I lost my words there. Could you ask again?", used, tokens_in, tokens_out)
            conversation.messages = messages
            conversation.exchanges += 1
            return Answer(True, reply, used, tokens_in, tokens_out, picture, mark)
        return Answer(False, "That took me more steps than I am allowed for one question. Could you ask it more simply?",
                      used, tokens_in, tokens_out)
