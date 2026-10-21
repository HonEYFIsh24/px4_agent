"""Regression contracts for offline reliability fault injection; no network or ROS."""
import importlib.util
import json
import unittest
from pathlib import Path

try:
    from langchain.agents import create_agent
except ImportError as exc:
    raise unittest.SkipTest("Offline reliability graph tests require Agent Python 3.11") from exc

MODULE = Path(__file__).resolve().parents[1] / "scripts/test_agent_reliability.py"
SPEC = importlib.util.spec_from_file_location("agent_reliability_runner", MODULE)
runner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(runner)


class AgentReliabilityTests(unittest.TestCase):
    def case(self, identifier):
        result = runner.run_case(identifier)
        failures = [a for a in result["assertions"] if not a["passed"]]
        self.assertTrue(result["passed"], json.dumps(failures, ensure_ascii=False, indent=2))
        self.assertEqual(result["source"], "offline_fault_injection_real_langchain_graph")
        self.assertTrue(result["observed"]["command"]["model_calls"])
        return result

    def test_connection_before_flight_never_moves_or_lands(self):
        self.case("connection_before_flight")

    def test_timeout_after_owned_takeoff_independently_lands(self):
        self.case("timeout_after_owned_takeoff")

    def test_failed_image_request_is_not_delivery_and_lands_owned_flight(self):
        self.case("connection_on_image_after_owned_takeoff")

    def test_camera_failure_blocks_following_model_movement(self):
        self.case("camera_delivery_failure_blocks_movement")

    def test_unreported_image_ttl_expiry_blocks_model_and_lands(self):
        self.case("unreported_frame_expired")

    def test_saved_report_allows_final_text_after_real_clock_expiry(self):
        self.case("saved_report_survives_frame_expiry")

    def test_invalid_tool_schema_blocks_automatic_flight_retry(self):
        self.case("invalid_tool_parameters_block_retry")

    def test_failed_mission_does_not_allow_model_to_resubmit(self):
        self.case("failed_mission_blocks_retry")

    def test_observation_does_not_land_someone_elses_airborne_flight(self):
        self.case("unowned_airborne_observation_failure")

    def test_operator_interrupt_prevents_automatic_recovery_landing(self):
        self.case("operator_interrupt_preserves_authority")

    def test_completed_full_flight_retains_failed_final_model_response_attribution(self):
        self.case("final_reply_connection_after_completed_flight")

    def test_saved_report_and_completed_return_survive_final_reply_connection_error(self):
        self.case("final_reply_error_after_saved_observation")

    def test_unconfirmed_accepted_flight_cannot_claim_execution_completed(self):
        self.case("final_reply_error_after_unconfirmed_acceptance")

    def test_failed_flight_is_not_relabelled_as_only_final_response_error(self):
        self.case("final_reply_error_after_failed_flight")

    def test_cancelled_flight_is_not_relabelled_as_completed(self):
        self.case("final_reply_error_after_cancelled_flight")

    def test_suite_labels_faults_offline_and_preserves_failed_task_status(self):
        result = runner.run_suite(["connection_before_flight", "saved_report_survives_frame_expiry"])
        self.assertTrue(result["passed"])
        self.assertEqual(result["passed_count"], 2)
        self.assertEqual(result["cases"][0]["observed"]["command"]["status"], "failed")
        self.assertIn("exceptions injected offline", result["scope"]["model"])
        self.assertIn("network send paths blocked", result["scope"]["transport"])


if __name__ == "__main__":
    unittest.main()
