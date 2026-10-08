import math
import os
import subprocess

import pytest


@pytest.fixture()
def route_map(tmp_path):
    path = tmp_path / "route_map.json"
    nodes = ",\n".join(
        '{"id": %d, "targetable": %s, '
        '"tag_pose": {"x": 0.0, "y": %.1f, "yaw": 0.0}}'
        % (tag_id, "true" if tag_id < 5 else "false", tag_id * 0.5)
        for tag_id in range(6)
    )
    forward = [
        '{"source": %d, "destination": %d, "cost": 1.0, '
        '"speed_limit": 0.25, "motion": "forward"}' % (tag_id, tag_id + 1)
        for tag_id in range(4)
    ]
    reverse = [
        '{"source": %d, "destination": %d, "cost": 1.0, '
        '"speed_limit": 0.20, "motion": "reverse"}' % (tag_id + 1, tag_id)
        for tag_id in range(4)
    ]
    path.write_text(
        '{"frame_id": "map", "nodes": [%s], "edges": [%s]}'
        % (nodes, ",\n".join(forward + reverse)),
        encoding="utf-8",
    )
    return path


def run_probe(route_map, *arguments, check=True):
    executable = os.environ["TAG_ROUTE_GRAPH_PROBE"]
    return subprocess.run(
        [executable, str(route_map), *map(str, arguments)],
        check=check,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def test_forward_reverse_and_unreachable_routes(route_map):
    forward = run_probe(route_map, "shortest", 0, 3).stdout.strip()
    reverse = run_probe(route_map, "shortest", 3, 0).stdout.strip()
    unreachable = run_probe(route_map, "shortest", 0, 5).stdout.strip()

    assert forward == "3 0:1,1:2,2:3"
    assert reverse == "3 3:2,2:1,1:0"
    assert unreachable == "UNREACHABLE"


def test_localization_only_tag_cannot_be_target(route_map):
    result = run_probe(
        route_map, "plan", 0.0, 2.5, math.pi / 2, 5, 0.1, 0.2,
        check=False,
    )
    assert result.returncode == 0
    assert result.stdout.strip() == "UNREACHABLE"
    assert "localization-only" in result.stderr

    target = [float(value) for value in
              run_probe(route_map, "target", 5).stdout.split()]
    assert target == pytest.approx([0.0, 2.5, math.pi / 2])


def test_plan_from_middle_of_directed_edge(route_map):
    forward = run_probe(
        route_map, "plan", 0.0, 0.75, math.pi / 2, 4, 0.1, 0.2
    ).stdout.strip()
    reverse = run_probe(
        route_map, "plan", 0.0, 0.75, math.pi / 2, 0, 0.1, 0.2
    ).stdout.strip()

    assert forward == "1:2,2:3,3:4"
    assert reverse == "2:1,1:0"


def test_start_behind_first_node_and_lateral_limit(route_map):
    route = run_probe(
        route_map, "plan", 0.0, -0.8, math.pi / 2, 3, 0.1, 0.2
    ).stdout.strip()
    points = run_probe(
        route_map, "remaining", 0.0, -0.8, math.pi / 2, 0, 1
    ).stdout.strip()
    off_route = run_probe(
        route_map, "plan", 0.25, -0.8, math.pi / 2, 3, 0.1, 0.2
    ).stdout.strip()

    assert route == "0:1,1:2,2:3"
    parsed_points = [
        tuple(float(value) for value in point.split(","))
        for point in points.split(";")
    ]
    expected_points = [
        (0.0, 0.0, math.pi / 2),
        (0.0, 0.5, math.pi / 2),
    ]
    assert len(parsed_points) == len(expected_points)
    for actual, expected in zip(parsed_points, expected_points):
        assert actual == pytest.approx(expected)
    assert off_route == "UNREACHABLE"


def test_route_line_uses_all_nodes_and_preserves_direction(route_map):
    forward = float(run_probe(
        route_map, "line_yaw", "0:1", "1:2", "2:3", "3:4"
    ).stdout)
    reverse = float(run_probe(
        route_map, "line_yaw", "4:3", "3:2", "2:1", "1:0"
    ).stdout)

    assert forward == pytest.approx(math.pi / 2)
    assert abs(reverse) == pytest.approx(math.pi / 2)


def test_invalid_input_reports_ros_error_without_exception(route_map, tmp_path):
    invalid_map = tmp_path / "invalid_route_map.json"
    invalid_map.write_text("{", encoding="utf-8")

    invalid_map_result = run_probe(invalid_map, "summary", check=False)
    invalid_edge_result = run_probe(
        route_map, "line_yaw", "invalid_edge", check=False
    )

    assert invalid_map_result.returncode == 2
    assert "Cannot parse route map" in invalid_map_result.stderr
    assert invalid_edge_result.returncode == 2
    assert "source:destination" in invalid_edge_result.stderr
