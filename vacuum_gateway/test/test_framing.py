import pytest

from vacuum_gateway import framing


def test_header_layout():
    # channel 1, type 0x10, length 3 as four big-endian bytes, then the message
    assert framing.encode(1, 0x10, b"abc", 100) == b"\x01\x10\x00\x00\x00\x03abc"


def test_round_trip():
    payload = bytes(range(256)) * 300  # 76,800 bytes: more than the old two-byte length allowed
    frame = framing.decode(framing.encode(0, 0x81, payload, 1 << 20), 1 << 20)
    assert frame == framing.Frame(0, 0x81, payload)


def test_empty_message_is_valid():
    assert framing.decode(framing.encode(1, 2, b"", 10), 10) == framing.Frame(1, 2, b"")


def test_encode_refuses_too_big():
    with pytest.raises(framing.FrameError, match="over the limit"):
        framing.encode(1, 1, b"x" * 11, 10)


def test_encode_refuses_values_over_one_byte():
    with pytest.raises(framing.FrameError):
        framing.encode(256, 1, b"", 10)
    with pytest.raises(framing.FrameError):
        framing.encode(1, -1, b"", 10)


def test_decode_refuses_short_input():
    with pytest.raises(framing.FrameError, match="shorter"):
        framing.decode(b"\x01\x02\x00", 10)


def test_decode_refuses_wrong_length():
    with pytest.raises(framing.FrameError, match="header says 3"):
        framing.decode(b"\x01\x02\x00\x00\x00\x03ab", 10)
    with pytest.raises(framing.FrameError, match="header says 1"):
        framing.decode(b"\x01\x02\x00\x00\x00\x01ab", 10)


def test_decode_refuses_too_big():
    with pytest.raises(framing.FrameError, match="over the limit"):
        framing.decode(b"\x01\x02\x00\x00\x00\x0b" + b"x" * 11, 10)
