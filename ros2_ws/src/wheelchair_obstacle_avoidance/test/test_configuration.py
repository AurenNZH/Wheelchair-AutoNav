from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def _launch_source():
    return (PACKAGE / "launch" / "obstacle_avoidance.launch.py").read_text()


def test_launch_is_reactive_only_and_uses_one_standalone_costmap():
    source = _launch_source()

    assert '"nav2_mapping.launch.py"' in source
    assert '"start_costmap": "true"' in source
    assert '"use_inflation": "true"' in source
    assert '"reactive_assistance_mode"' in source
    assert 'default_value="enforce"' in source
    assert 'choices=["disabled", "shadow", "enforce"]' in source
    assert '"maximum_assist", default_value="0.577350269"' in source
    assert '"; maximum assist ratio="' in source
    assert '"turn_clearance_radius_m", default_value="0.45"' in source
    assert 'DeclareLaunchArgument("max_intent_age_s", default_value="1.00")' in source
    assert '"turn_clearance_radius_m": LaunchConfiguration(' in source
    assert '"max_intent_age_s": LaunchConfiguration("max_intent_age_s")' in source

    for legacy in (
        "nav2_waypoint",
        "local_avoidance_planner",
        "planner_server",
        "lifecycle_manager",
        "RewrittenYaml",
        "planner_search_budget_ms",
        "discard_after_ms",
    ):
        assert legacy not in source


def test_actuation_and_udp_gates_remain_fail_safe():
    source = _launch_source()

    assert 'DeclareLaunchArgument("enable_motion", default_value="false")' in source
    assert '"geometry_calibrated", default_value="false"' in source
    assert 'DeclareLaunchArgument("enable_udp", default_value="false")' in source
    for argument in (
        "enable_motion",
        "geometry_calibrated",
        "enable_udp",
        "bind_address",
        "pi_address",
        "allowed_pi_address",
    ):
        assert '"%s": LaunchConfiguration("%s")' % (argument, argument) in source


def test_package_has_no_waypoint_runtime_or_nav2_planner_dependencies():
    setup_source = (PACKAGE / "setup.py").read_text()
    manifest = (PACKAGE / "package.xml").read_text()

    assert "local_avoidance_planner" not in setup_source
    assert "config/*.yaml" not in setup_source
    for dependency in (
        "nav2_planner",
        "smac_planner",
        "nav2_lifecycle_manager",
        "nav2_msgs",
        "nav2_common",
    ):
        assert dependency not in manifest


def test_only_reactive_package_module_remains():
    modules = {
        path.name
        for path in (PACKAGE / "wheelchair_obstacle_avoidance").glob("*.py")
    }
    assert modules == {"__init__.py"}
