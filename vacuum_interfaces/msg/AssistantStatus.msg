# What the assistant is doing: /assistant/status, on change and twice a second.
bool busy               # answering a question (or doing what it was asked)
string conversation     # whose question, as AskAssistant names it; empty when not busy
string doing            # what it is doing right now, in a few words ("going to the kitchen"); empty when not busy
