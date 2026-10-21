"""Select the active architecture without exposing provider credentials."""
import os
from urllib.parse import urlsplit

from vision import vision_configuration


def runtime_configuration(env_file):
    from dotenv import dotenv_values
    values = {**dotenv_values(env_file), **os.environ}
    architecture = values.get("AGENT_ARCHITECTURE", "single_vlm")
    if architecture not in ("single_vlm", "dual_model"):
        raise RuntimeError("AGENT_ARCHITECTURE must be single_vlm or dual_model")
    primary = {
        "api_key": values.get("LLM_API_KEY") or values.get("OPENAI_API_KEY") or values.get("DASHSCOPE_API_KEY"),
        "base_url": values.get("LLM_BASE_URL") or values.get("BASE_URL") or values.get("OPENAI_BASE_URL"),
        "model": values.get("LLM_MODEL") or values.get("MODEL") or "qwen-plus",
    }
    vision = vision_configuration(env_file, primary)
    if architecture == "dual_model":
        if not primary["api_key"]:
            raise RuntimeError("双模型 Agent 未配置 LLM_API_KEY")
        return architecture, primary, vision
    url = values.get("VLM_BASE_URL") or values.get("VISION_BASE_URL") or primary.get("base_url")
    model = values.get("VLM_MODEL") or values.get("VISION_MODEL")
    if not model and "vl" in primary["model"].lower():
        model = primary["model"]
    if not model and (urlsplit(url or "").hostname or "").endswith("aliyuncs.com"):
        model = "qwen3-vl-plus"
    key = values.get("VLM_API_KEY") or values.get("VISION_API_KEY") or primary.get("api_key")
    if not key or not model:
        raise RuntimeError("单 VLM Agent 未配置；请设置支持图像与工具调用的 VLM_MODEL、VLM_BASE_URL、VLM_API_KEY")
    active = {"api_key": key, "base_url": url, "model": model}
    return architecture, active, active.copy()
