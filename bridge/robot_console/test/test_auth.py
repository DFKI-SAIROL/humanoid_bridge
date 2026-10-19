"""Token file, cookie and Origin checks."""

from __future__ import annotations

import stat

from robot_console import auth


def test_token_is_made_once_with_mode_600(tmp_path):
    path = tmp_path / "cfg" / "token"
    first = auth.load_or_create_token(path)
    assert len(first) >= 24
    assert stat.S_IMODE(path.stat().st_mode) == 0o600
    assert stat.S_IMODE(path.parent.stat().st_mode) == 0o700
    assert auth.load_or_create_token(path) == first


def test_cookie_round_trip():
    header = auth.cookie_header("abc")
    assert "HttpOnly" in header and "SameSite=Strict" in header
    assert auth.token_ok({"Cookie": f"{auth.COOKIE}=abc"}, "abc")
    assert auth.token_ok({"Cookie": f"other=1; {auth.COOKIE}=abc"}, "abc")
    assert not auth.token_ok({"Cookie": f"{auth.COOKIE}=abd"}, "abc")
    assert not auth.token_ok({}, "abc")
    assert not auth.token_ok({"Cookie": "\x00garbage;;=="}, "abc")


def test_query_token():
    assert auth.query_token_ok("token=abc", "abc")
    assert auth.query_token_ok("x=1&token=abc", "abc")
    assert not auth.query_token_ok("token=abd", "abc")
    assert not auth.query_token_ok("", "abc")


def test_origin():
    assert auth.origin_ok({"Host": "10.0.0.5:8080"})
    assert auth.origin_ok({"Host": "10.0.0.5:8080", "Origin": "http://10.0.0.5:8080"})
    assert not auth.origin_ok({"Host": "10.0.0.5:8080", "Origin": "http://evil.example"})
