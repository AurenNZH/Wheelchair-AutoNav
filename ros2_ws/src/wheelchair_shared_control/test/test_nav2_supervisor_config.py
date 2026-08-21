from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import yaml

from wheelchair_shared_control.safety import SafetyConfig, SafetyDecision
from wheelchair_shared_control.supervisor_node import (
    LEGACY_MAP_STAMP,
    NAV2_LIVE,
    SafetySupervisorNode,
    evaluate_freshness,
    safety_diagnostic_values,
)


def _parameters():
    path = Path(__file__).parents[1] / "config" / "shared_control.yaml"
    document = yaml.safe_load(path.read_text())
    return document["/**"]["ros__parameters"]


def test_production_defaults_to_weighted_nav2_costs():
    parameters = _parameters()

    assert parameters["front_costmap_topic"] == "/nav2_front_costmap"
    assert parameters["source_header_topic"] == (
        "/artifact_filter/source_header"
    )
    assert parameters["freshness_mode"] == NAV2_LIVE
    assert parameters["max_map_age_s"] == 0.5
    assert parameters["max_source_age_s"] == 0.5
    assert parameters["slow_cost_threshold"] == 1
    assert parameters["stop_cost_threshold"] == 99
    assert parameters["slow_forward_limit"] == 0.30
    assert parameters["reverse_limit"] == 0.65
    assert parameters["min_steering"] == -0.577350269
    assert parameters["max_steering"] == 0.577350269
    assert parameters["forward_cone_half_angle_deg"] == 30.0
    assert parameters["enable_motion"] is False
    assert parameters["geometry_calibrated"] is False


def test_launch_exposes_costmap_topic_and_thresholds():
    path = Path(__file__).parents[1] / "launch" / "shared_control.launch.py"
    source = path.read_text()

    assert '"front_costmap_topic"' in source
    assert 'default_value="/nav2_front_costmap"' in source
    assert 'default_value="/artifact_filter/source_header"' in source
    assert '"freshness_mode", default_value="nav2_live"' in source
    assert '"max_map_age_s", default_value="0.50"' in source
    assert '"max_source_age_s", default_value="0.50"' in source
    assert '"slow_cost_threshold", default_value="1"' in source
    assert '"stop_cost_threshold", default_value="99"' in source
    assert '"forward_cone_half_angle_deg", default_value="30.0"' in source


def test_replay_keeps_legacy_front_topic_explicit():
    path = Path(__file__).parents[1] / "launch" / "intent_replay.launch.py"
    source = path.read_text()

    assert '"front_costmap_topic": "/front_costmap"' in source
    assert '"freshness_mode": "legacy_map_stamp"' in source


def _live_freshness(**changes):
    arguments = {
        "mode": NAV2_LIVE,
        "now_ros_ns": 10_000_000_000,
        "now_monotonic_ns": 20_000_000_000,
        "map_stamp_ns": 0,
        "map_received_monotonic_ns": 19_900_000_000,
        "source_stamp_ns": 9_800_000_000,
        "max_source_age_s": 0.5,
        "max_future_source_offset_s": 0.1,
    }
    arguments.update(changes)
    return evaluate_freshness(**arguments)


def test_nav2_live_accepts_zero_map_stamp_with_both_watchdogs_fresh():
    status = _live_freshness()

    assert status.failure_reason is None
    assert status.map_age_basis == "receipt_time"
    assert status.map_age_s == 0.1
    assert status.source_age_s == 0.2


def test_nav2_live_fails_closed_for_missing_or_invalid_source():
    missing = _live_freshness(source_stamp_ns=None)
    invalid = _live_freshness(source_stamp_ns=0)
    future = _live_freshness(source_stamp_ns=10_200_000_000)
    stale = _live_freshness(source_stamp_ns=9_400_000_000)

    assert missing.failure_reason == "missing_source_heartbeat"
    assert invalid.failure_reason == "invalid_source_timestamp"
    assert future.failure_reason == "future_source_timestamp"
    assert stale.failure_reason == "stale_source"


def test_nav2_live_allows_small_source_clock_offset():
    status = _live_freshness(source_stamp_ns=10_050_000_000)

    assert status.failure_reason is None
    assert status.source_age_s == 0.0


def test_nav2_live_rejects_missing_or_reversed_map_receipt_time():
    missing = _live_freshness(map_received_monotonic_ns=0)
    reversed_time = _live_freshness(
        map_received_monotonic_ns=20_100_000_000
    )

    assert missing.failure_reason == "missing_map_receipt"
    assert reversed_time.failure_reason == "invalid_map_receipt_time"


def test_legacy_mode_retains_map_header_stamp_semantics():
    invalid = _live_freshness(mode=LEGACY_MAP_STAMP)
    valid = _live_freshness(
        mode=LEGACY_MAP_STAMP,
        map_stamp_ns=9_800_000_000,
        source_stamp_ns=None,
    )

    assert invalid.failure_reason == "invalid_map_timestamp"
    assert valid.failure_reason is None
    assert valid.map_age_basis == "map_header_stamp"
    assert valid.map_age_s == 0.2


def test_supervisor_retains_weighted_grid_values():
    msg = OccupancyGrid()
    msg.header.frame_id = "base_link"
    msg.info.width = 2
    msg.info.height = 2
    msg.info.resolution = 0.1
    msg.info.origin.position.y = -0.1
    msg.info.origin.orientation.w = 1.0
    msg.data = [0, 50, 99, 100]

    costmap = SafetySupervisorNode._grid_costmap(msg)

    assert costmap.costs.tolist() == [[0, 50], [99, 100]]


def test_cost_decision_diagnostics_expose_calibration_evidence():
    config = SafetyConfig(
        slow_cost_threshold=5,
        stop_cost_threshold=90,
    )
    decision = SafetyDecision(
        decision=1,
        permitted_forward=0.2,
        permitted_steering=0.0,
        reason="nav2_cost_slow",
        nearest_path_distance_m=0.8,
        maximum_path_cost=75,
        nearest_slow_cost_distance_m=0.8,
        nearest_stop_cost_distance_m=None,
        path_cost_valid=True,
    )

    values = {
        item.key: item.value
        for item in safety_diagnostic_values(decision, config, 12.5, 1.25)
    }

    assert values["maximum_path_cost"] == "75"
    assert values["nearest_slow_cost_distance_m"] == "0.800"
    assert values["nearest_stop_cost_distance_m"] == "none"
    assert values["slow_cost_threshold"] == "5"
    assert values["stop_cost_threshold"] == "90"
    assert values["path_cost_valid"] == "True"
    assert values["freshness_mode"] == NAV2_LIVE
    assert values["map_age_basis"] == "receipt_time"
    assert values["source_age_ms"] == "none"
