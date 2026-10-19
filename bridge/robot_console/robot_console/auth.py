"""The page's one secret: a token in a file, carried by a cookie."""

from __future__ import annotations

import hmac
import os
import secrets
from http.cookies import SimpleCookie
from pathlib import Path
from urllib.parse import urlsplit

COOKIE = "robot_console_token"
DEFAULT_TOKEN_FILE = Path("~/.config/robot_console/token").expanduser()


def load_or_create_token(path: Path = DEFAULT_TOKEN_FILE) -> str:
    """The token in `path`; made once (mode 600) and kept, so a phone bookmark keeps working."""
    path = Path(path)
    if path.is_file():
        token = path.read_text().strip()
        if token:
            return token
    path.parent.mkdir(parents=True, exist_ok=True)
    os.chmod(path.parent, 0o700)
    token = secrets.token_urlsafe(24)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as handle:
        handle.write(token + "\n")
    return token


def cookie_header(token: str) -> str:
    """Set-Cookie value for a granted token."""
    return f"{COOKIE}={token}; Path=/; HttpOnly; SameSite=Strict; Max-Age=31536000"


def token_ok(headers, token: str) -> bool:
    """True when the request's cookie carries `token`."""
    raw = headers.get("Cookie")
    if not raw:
        return False
    jar = SimpleCookie()
    try:
        jar.load(raw)
    except Exception:  # noqa: BLE001 - a malformed cookie is just "no token"
        return False
    morsel = jar.get(COOKIE)
    return morsel is not None and hmac.compare_digest(morsel.value, token)


def query_token_ok(query: str, token: str) -> bool:
    """True when `?token=` in the query string is `token`."""
    for part in query.split("&"):
        key, _, value = part.partition("=")
        if key == "token":
            return hmac.compare_digest(value, token)
    return False


def origin_ok(headers) -> bool:
    """No Origin (curl, same-origin GET) or an Origin naming this host."""
    origin = headers.get("Origin")
    if not origin:
        return True
    return urlsplit(origin).netloc == headers.get("Host", "")
