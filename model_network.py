"""Opt-in proxy configuration scoped to the Agent process, not ROS or PX4."""
import os
from urllib.parse import urlsplit

from dotenv import dotenv_values


def configure_agent_network(env_file):
    proxy = os.environ.get("UAV_MODEL_PROXY") or dotenv_values(env_file).get("UAV_MODEL_PROXY")
    if not proxy:
        return
    parsed = urlsplit(proxy)
    if parsed.scheme not in ("http", "https") or not parsed.hostname:
        raise ValueError("UAV_MODEL_PROXY 必须为有效的 HTTP/HTTPS 代理地址")
    # Both cases are set because urllib and httpx can prefer lowercase values.
    for name in ("HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy"):
        os.environ[name] = proxy
    bypass = {item.strip() for name in ("NO_PROXY", "no_proxy")
              for item in os.environ.get(name, "").split(",") if item.strip()}
    bypass.update(("127.0.0.1", "localhost", "::1"))
    for name in ("NO_PROXY", "no_proxy"):
        os.environ[name] = ",".join(sorted(bypass))
