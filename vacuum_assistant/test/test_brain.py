"""The conversation loop, against a stand-in for the language model that answers from a script."""

from types import SimpleNamespace

import anthropic
import httpx
import pytest

from vacuum_assistant import brain
from vacuum_assistant.tools import ToolOutput


def text(t):
    return SimpleNamespace(type="text", text=t)


def tool_use(name, args=None, id="t1"):
    return SimpleNamespace(type="tool_use", name=name, input=args or {}, id=id)


def reply(stop, *blocks):
    return SimpleNamespace(stop_reason=stop, content=list(blocks), usage=SimpleNamespace(input_tokens=100, output_tokens=10))


class FakeModel:
    """Plays back replies in order and keeps every request it was sent."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.requests = []
        self.messages = self
        self.beta = SimpleNamespace(messages=self)

    def create(self, **request):
        self.requests.append({**request, "messages": list(request["messages"])})
        out = self.replies.pop(0)
        if isinstance(out, Exception):
            raise out
        return out


def make(model, tools=None, **options):
    calls = []

    def run_tool(name, args):
        calls.append((name, args))
        text, error = (tools or {}).get(name, ("no such tool", True))
        return ToolOutput(text, error)

    return brain.Brain(model, "you are a robot", [{"name": "get_status"}], run_tool, log=lambda _t: None, **options), calls


def test_a_plain_answer_is_kept_in_the_conversation():
    model = FakeModel(reply("end_turn", text("I am parked on my dock.")))
    b, calls = make(model)
    c = brain.Conversation()
    answer = b.ask(c, "where are you?")
    assert answer.ok and answer.text == "I am parked on my dock." and answer.tools_used == [] and not calls
    assert [m["role"] for m in c.messages] == ["user", "assistant"] and c.exchanges == 1
    request = model.requests[0]
    assert request["model"] == brain.MODEL and request["output_config"] == {"effort": "low"}
    assert request["system"][0]["cache_control"] == {"type": "ephemeral"}


def test_tools_are_run_and_their_results_go_back_in_one_message():
    model = FakeModel(
        reply("tool_use", text("Let me check."), tool_use("get_status", id="a"), tool_use("get_cleaning_history", {"count": 2}, id="b")),
        reply("end_turn", text("You cleaned this morning.")),
    )
    b, calls = make(model, {"get_status": ('{"mode":"docked"}', False), "get_cleaning_history": ("broken", True)})
    c = brain.Conversation()
    answer = b.ask(c, "is the floor clean?")
    assert answer.ok and answer.text == "You cleaned this morning."
    assert answer.tools_used == ["get_status", "get_cleaning_history"]
    assert calls == [("get_status", {}), ("get_cleaning_history", {"count": 2})]
    results = model.requests[1]["messages"][-1]
    assert results["role"] == "user" and [r["tool_use_id"] for r in results["content"]] == ["a", "b"]
    assert "is_error" not in results["content"][0] and results["content"][1]["is_error"] is True
    assert results["content"][0]["content"] == [{"type": "text", "text": '{"mode":"docked"}'}]
    assert answer.input_tokens == 200 and answer.output_tokens == 20


def test_the_next_question_carries_the_conversation_on():
    model = FakeModel(reply("end_turn", text("Hello.")), reply("end_turn", text("Still here.")))
    b, _ = make(model)
    c = brain.Conversation()
    b.ask(c, "hi")
    b.ask(c, "are you there?")
    assert [m["role"] for m in model.requests[1]["messages"]] == ["user", "assistant", "user"]


def test_a_tool_that_raises_is_an_error_result_not_a_crash():
    model = FakeModel(reply("tool_use", tool_use("get_status")), reply("end_turn", text("I could not check.")))

    def broken(name, args):
        raise RuntimeError("no robot state")

    b = brain.Brain(model, "s", [], broken, log=lambda _t: None)
    answer = b.ask(brain.Conversation(), "status?")
    assert answer.ok
    result = model.requests[1]["messages"][-1]["content"][0]
    assert result["is_error"] is True and "no robot state" in result["content"][-1]["text"]


def _status_error(cls, code):
    request = httpx.Request("POST", "https://example.invalid")
    return cls("nope", response=httpx.Response(code, request=request), body=None)


@pytest.mark.parametrize("error, words", [
    (_status_error(anthropic.AuthenticationError, 401), "API key was not accepted"),
    (_status_error(anthropic.RateLimitError, 429), "Try again in a minute"),
    (_status_error(anthropic.InternalServerError, 500), "(500)"),
    (anthropic.APIConnectionError(request=httpx.Request("POST", "https://example.invalid")), "cannot reach the internet"),
])
def test_a_failure_is_said_plainly_and_the_conversation_is_left_as_it_was(error, words):
    b, _ = make(FakeModel(error))
    c = brain.Conversation()
    answer = b.ask(c, "hi")
    assert not answer.ok and words in answer.text
    assert c.messages == [] and c.exchanges == 0


def test_a_refusal_is_not_kept():
    b, _ = make(FakeModel(reply("refusal")))
    c = brain.Conversation()
    answer = b.ask(c, "something odd")
    assert not answer.ok and answer.text and c.messages == []


def test_an_answer_cut_off_inside_a_tool_call_is_not_kept():
    b, calls = make(FakeModel(reply("max_tokens", text("Checking"), tool_use("get_status"))))
    c = brain.Conversation()
    answer = b.ask(c, "status?")
    assert not answer.ok and not calls and c.messages == []


def test_too_many_rounds_end_the_question():
    model = FakeModel(*[reply("tool_use", tool_use("get_status", id=str(i))) for i in range(3)])
    b, calls = make(model, {"get_status": ("{}", False)}, max_rounds=3)
    c = brain.Conversation()
    answer = b.ask(c, "loop")
    assert not answer.ok and "more steps" in answer.text and len(calls) == 3 and c.messages == []


def test_a_long_conversation_starts_afresh():
    model = FakeModel(reply("end_turn", text("Fresh.")))
    b, _ = make(model)
    c = brain.Conversation(messages=[{"role": "user", "content": "old"}, {"role": "assistant", "content": "old"}],
                           exchanges=brain.MAX_EXCHANGES)
    b.ask(c, "new")
    assert model.requests[0]["messages"] == [{"role": "user", "content": "new"}]


def test_the_fallback_is_asked_for_only_when_wanted():
    with_it = FakeModel(reply("end_turn", text("a")))
    make(with_it)[0].ask(brain.Conversation(), "hi")
    assert with_it.requests[0]["betas"] == [brain.FALLBACK_BETA] and with_it.requests[0]["extra_body"] == {"fallbacks": "default"}
    without = FakeModel(reply("end_turn", text("a")))
    make(without, fallback=False)[0].ask(brain.Conversation(), "hi")
    assert "betas" not in without.requests[0] and "extra_body" not in without.requests[0]


def test_an_empty_reply_is_not_kept():
    b, _ = make(FakeModel(reply("end_turn")))
    c = brain.Conversation()
    answer = b.ask(c, "hi")
    assert not answer.ok and c.messages == []


def test_any_other_error_still_gives_an_answer():
    b, _ = make(FakeModel(TypeError("unexpected keyword")))
    c = brain.Conversation()
    answer = b.ask(c, "hi")
    assert not answer.ok and "went wrong" in answer.text and c.messages == []


def test_a_picture_goes_to_the_model_and_with_the_answer_and_so_does_a_place():
    model = FakeModel(reply("tool_use", tool_use("look")), reply("end_turn", text("I see a sofa.")))

    def run_tool(name, args):
        return ToolOutput('{"robot":{}}', pictures=[b"\xff\xd8jpeg"], mark=(1.5, -2.0))

    seen = []
    b = brain.Brain(model, "s", [], run_tool, log=lambda _t: None, on_tool=seen.append)
    answer = b.ask(brain.Conversation(), "what do you see?")
    assert answer.ok and answer.picture_jpeg == b"\xff\xd8jpeg" and answer.mark == (1.5, -2.0) and seen == ["look"]
    content = model.requests[1]["messages"][-1]["content"][0]["content"]
    assert content[0]["type"] == "image" and content[0]["source"]["media_type"] == "image/jpeg" and content[1]["type"] == "text"
