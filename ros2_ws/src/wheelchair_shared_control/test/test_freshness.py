import unittest
from dataclasses import replace

from wheelchair_shared_control.freshness import (
    LEGACY_MAP_STAMP,
    NAV2_LIVE,
    FreshnessInputs,
    FreshnessPolicy,
    evaluate_freshness,
    validate_freshness_policy,
)


class FreshnessTests(unittest.TestCase):
    def setUp(self):
        self.inputs = FreshnessInputs(
            now_ros_ns=10_000_000_000,
            now_monotonic_ns=20_000_000_000,
            intent_stamp_ns=9_900_000_000,
            map_available=True,
            map_stamp_ns=0,
            map_received_monotonic_ns=19_900_000_000,
            source_stamp_ns=9_800_000_000,
        )
        self.policy = FreshnessPolicy()

    def evaluate(self, *, policy=None, **changes):
        return evaluate_freshness(
            replace(self.inputs, **changes),
            self.policy if policy is None else policy,
        )

    def test_nav2_live_accepts_zero_map_stamp_when_watchdogs_are_fresh(self):
        status = self.evaluate()

        self.assertIsNone(status.failure_reason)
        self.assertEqual(status.intent_age_s, 0.1)
        self.assertEqual(status.map_age_s, 0.1)
        self.assertEqual(status.source_age_s, 0.2)
        self.assertEqual(status.map_age_basis, "receipt_time")

    def test_intent_checks_precede_map_checks(self):
        fixtures = (
            (None, "missing_intent"),
            (0, "invalid_intent_timestamp"),
            (10_100_000_000, "invalid_intent_timestamp"),
            (9_700_000_000, "stale_intent"),
        )
        for stamp_ns, expected in fixtures:
            with self.subTest(stamp_ns=stamp_ns):
                status = self.evaluate(
                    intent_stamp_ns=stamp_ns,
                    map_available=False,
                )
                self.assertEqual(status.failure_reason, expected)

    def test_missing_map_follows_valid_intent(self):
        status = self.evaluate(map_available=False)

        self.assertEqual(status.failure_reason, "missing_map")

    def test_nav2_live_fails_closed_for_source_timestamp_errors(self):
        fixtures = (
            (None, "missing_source_heartbeat"),
            (0, "invalid_source_timestamp"),
            (10_200_000_000, "future_source_timestamp"),
            (9_400_000_000, "stale_source"),
        )
        for stamp_ns, expected in fixtures:
            with self.subTest(stamp_ns=stamp_ns):
                status = self.evaluate(source_stamp_ns=stamp_ns)
                self.assertEqual(status.failure_reason, expected)

    def test_small_source_clock_offset_is_allowed_and_clamped(self):
        status = self.evaluate(source_stamp_ns=10_050_000_000)

        self.assertIsNone(status.failure_reason)
        self.assertEqual(status.source_age_s, 0.0)

    def test_nav2_live_rejects_missing_or_reversed_receipt_time(self):
        missing = self.evaluate(map_received_monotonic_ns=0)
        reversed_time = self.evaluate(
            map_received_monotonic_ns=20_100_000_000
        )

        self.assertEqual(missing.failure_reason, "missing_map_receipt")
        self.assertEqual(
            reversed_time.failure_reason,
            "invalid_map_receipt_time",
        )

    def test_source_failure_precedes_stale_live_map(self):
        status = self.evaluate(
            map_received_monotonic_ns=19_000_000_000,
            source_stamp_ns=9_000_000_000,
        )
        stale_map = self.evaluate(
            map_received_monotonic_ns=19_000_000_000
        )

        self.assertEqual(status.failure_reason, "stale_source")
        self.assertEqual(stale_map.failure_reason, "stale_map")

    def test_legacy_mode_retains_map_header_semantics(self):
        policy = replace(self.policy, mode=LEGACY_MAP_STAMP)
        invalid = self.evaluate(policy=policy)
        valid = self.evaluate(
            policy=policy,
            map_stamp_ns=9_800_000_000,
            source_stamp_ns=None,
        )
        future = self.evaluate(
            policy=policy,
            map_stamp_ns=10_100_000_000,
        )
        stale = self.evaluate(
            policy=policy,
            map_stamp_ns=9_000_000_000,
        )

        self.assertEqual(invalid.failure_reason, "invalid_map_timestamp")
        self.assertIsNone(valid.failure_reason)
        self.assertEqual(valid.map_age_s, 0.2)
        self.assertEqual(valid.map_age_basis, "map_header_stamp")
        self.assertEqual(future.failure_reason, "invalid_map_age")
        self.assertEqual(stale.failure_reason, "stale_map")

    def test_startup_validation_preserves_supported_settings(self):
        validate_freshness_policy(self.policy)
        validate_freshness_policy(
            replace(self.policy, mode=LEGACY_MAP_STAMP)
        )
        invalid = (
            replace(self.policy, mode="unknown"),
            replace(self.policy, max_source_age_s=0.0),
            replace(self.policy, max_future_source_offset_s=-0.1),
        )
        for policy in invalid:
            with self.subTest(policy=policy), self.assertRaises(ValueError):
                validate_freshness_policy(policy)

    def test_mode_constants_remain_stable(self):
        self.assertEqual(NAV2_LIVE, "nav2_live")
        self.assertEqual(LEGACY_MAP_STAMP, "legacy_map_stamp")


if __name__ == "__main__":
    unittest.main()
