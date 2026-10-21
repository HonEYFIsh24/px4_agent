"""Architecture selection and provider precedence; no model or flight calls."""
import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from agent_config import runtime_configuration


DASHSCOPE = "https://dashscope.aliyuncs.com/compatible-mode/v1"
PRIMARY = {
    "LLM_API_KEY": "fixture-primary-key",
    "LLM_BASE_URL": DASHSCOPE,
    "LLM_MODEL": "deepseek-v3.2",
}
VISION = {
    "VISION_API_KEY": "fixture-vision-key",
    "VISION_BASE_URL": "https://vision.example.test/v1",
    "VISION_MODEL": "fixture-vision-model",
}
VLM = {
    "VLM_API_KEY": "fixture-vlm-key",
    "VLM_BASE_URL": "https://vlm.example.test/v1",
    "VLM_MODEL": "fixture-vlm-model",
}


@unittest.skipUnless(importlib.util.find_spec("dotenv"), "python-dotenv is unavailable in this interpreter")
class AgentConfigurationTests(unittest.TestCase):
    def configure(self, file_values, environ=None):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / ".env"
            original = "".join("{}={}\n".format(key, value) for key, value in file_values.items())
            path.write_text(original, encoding="utf-8")
            with mock.patch.dict(os.environ, environ or {}, clear=True):
                result = runtime_configuration(str(path))
            self.assertEqual(path.read_text(encoding="utf-8"), original,
                             "Reading architecture configuration must not alter the operator's env file")
            return result

    def assert_single(self, result, expected):
        architecture, active, vision = result
        self.assertEqual(architecture, "single_vlm")
        self.assertEqual(active, expected)
        self.assertEqual(vision, expected)
        self.assertIsNot(active, vision)

    def test_legacy_dashscope_text_configuration_selects_single_vlm_without_editing_file(self):
        self.assert_single(self.configure(PRIMARY), {
            "api_key": PRIMARY["LLM_API_KEY"], "base_url": DASHSCOPE, "model": "qwen3-vl-plus",
        })

    def test_explicit_vlm_values_override_primary_and_independent_vision(self):
        self.assert_single(self.configure({**PRIMARY, **VISION, **VLM}), {
            "api_key": VLM["VLM_API_KEY"], "base_url": VLM["VLM_BASE_URL"], "model": VLM["VLM_MODEL"],
        })

    def test_vision_values_supply_single_vlm_when_vlm_values_are_absent(self):
        self.assert_single(self.configure({**PRIMARY, **VISION}), {
            "api_key": VISION["VISION_API_KEY"], "base_url": VISION["VISION_BASE_URL"],
            "model": VISION["VISION_MODEL"],
        })

    def test_environment_overrides_same_names_from_file(self):
        override = {"VLM_API_KEY": "fixture-environment-key", "VLM_BASE_URL": "https://env.example.test/v1",
                    "VLM_MODEL": "fixture-environment-model"}
        self.assert_single(self.configure({**PRIMARY, **VLM}, override), {
            "api_key": override["VLM_API_KEY"], "base_url": override["VLM_BASE_URL"],
            "model": override["VLM_MODEL"],
        })

    def test_dual_model_retains_primary_and_independent_vision(self):
        architecture, primary, vision = self.configure({**PRIMARY, **VISION, **VLM,
                                                        "AGENT_ARCHITECTURE": "dual_model"})
        self.assertEqual(architecture, "dual_model")
        self.assertEqual(primary, {"api_key": PRIMARY["LLM_API_KEY"], "base_url": DASHSCOPE,
                                   "model": "deepseek-v3.2"})
        self.assertEqual(vision, {"api_key": VISION["VISION_API_KEY"], "base_url": VISION["VISION_BASE_URL"],
                                  "model": VISION["VISION_MODEL"]})

    def test_invalid_architecture_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "AGENT_ARCHITECTURE"):
            self.configure({**PRIMARY, "AGENT_ARCHITECTURE": "unsupported"})

    def test_non_dashscope_text_model_requires_an_explicit_image_model(self):
        with self.assertRaisesRegex(RuntimeError, "VLM_MODEL"):
            self.configure({**PRIMARY, "LLM_BASE_URL": "https://text.example.test/v1"})

    def test_vlm_values_work_without_primary_or_vision_credentials(self):
        self.assert_single(self.configure(VLM), {
            "api_key": VLM["VLM_API_KEY"], "base_url": VLM["VLM_BASE_URL"], "model": VLM["VLM_MODEL"],
        })

    def test_partial_vision_provider_values_are_inherited_with_explicit_vlm_model(self):
        self.assert_single(self.configure({**PRIMARY, "VLM_MODEL": VLM["VLM_MODEL"],
                                          "VISION_API_KEY": VISION["VISION_API_KEY"],
                                          "VISION_BASE_URL": VISION["VISION_BASE_URL"]}), {
            "api_key": VISION["VISION_API_KEY"], "base_url": VISION["VISION_BASE_URL"],
            "model": VLM["VLM_MODEL"],
        })

    def test_primary_vl_model_is_inherited_before_dashscope_default(self):
        self.assert_single(self.configure({**PRIMARY, "LLM_MODEL": "qwen-vl-max"}), {
            "api_key": PRIMARY["LLM_API_KEY"], "base_url": DASHSCOPE, "model": "qwen-vl-max",
        })


if __name__ == "__main__":
    unittest.main()
