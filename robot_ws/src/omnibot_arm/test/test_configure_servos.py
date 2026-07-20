"""
Unit tests for configure_servos.py — the SO-101 servo bring-up tool.

Only the pure, hardware-free helpers are covered here: ID validation, the
home_ticks YAML rendering that gets pasted into arm_params.yaml, and the
jog-delta clamp that stops a typo from slamming a joint into its stop.
No rclpy, no lerobot, no serial port required.
"""

import os
import sys

import pytest

# Add scripts/ to sys.path so the module is importable without being installed.
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
)

import configure_servos as cs  # noqa: E402 — must come after sys.path patch


# ---------------------------------------------------------------------------
# validate_id
# ---------------------------------------------------------------------------


class TestValidateId:
    def test_accepts_min(self):
        assert cs.validate_id(1) == 1

    def test_accepts_max(self):
        assert cs.validate_id(253) == 253

    def test_rejects_zero(self):
        with pytest.raises(ValueError):
            cs.validate_id(0)

    def test_rejects_above_range(self):
        with pytest.raises(ValueError):
            cs.validate_id(254)


# ---------------------------------------------------------------------------
# joint_for_id
# ---------------------------------------------------------------------------


class TestJointForId:
    def test_known_ids_map_to_joint_names(self):
        assert cs.joint_for_id(1) == "arm_shoulder_pan"
        assert cs.joint_for_id(6) == "arm_gripper"

    def test_unknown_id_falls_back(self):
        assert cs.joint_for_id(99) == "id 99"


# ---------------------------------------------------------------------------
# home_ticks_yaml
# ---------------------------------------------------------------------------


class TestHomeTicksYaml:
    def test_contains_all_six_values(self):
        out = cs.home_ticks_yaml([2048, 2048, 2048, 2048, 2048, 2048])
        assert "home_ticks: [2048, 2048, 2048, 2048, 2048, 2048]" in out

    def test_coerces_floats_to_int(self):
        out = cs.home_ticks_yaml([2048.7, 100.0, 0, 0, 0, 0])
        assert "home_ticks: [2048, 100, 0, 0, 0, 0]" in out

    def test_annotates_each_joint(self):
        out = cs.home_ticks_yaml([1, 2, 3, 4, 5, 6])
        for name in cs.JOINT_NAMES:
            assert name in out

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError):
            cs.home_ticks_yaml([2048, 2048])


# ---------------------------------------------------------------------------
# clamp_delta
# ---------------------------------------------------------------------------


class TestClampDelta:
    def test_within_limit_unchanged(self):
        assert cs.clamp_delta(200) == 200

    def test_positive_over_limit_clamped(self):
        assert cs.clamp_delta(99999) == cs.TICKS_PER_REV // 4

    def test_negative_over_limit_clamped(self):
        assert cs.clamp_delta(-99999) == -(cs.TICKS_PER_REV // 4)

    def test_float_coerced_to_int(self):
        assert cs.clamp_delta(50.9) == 50


# ---------------------------------------------------------------------------
# CLI wiring — argument parsing without touching hardware
# ---------------------------------------------------------------------------


class TestParser:
    def test_set_id_requires_from_and_to(self):
        parser = cs.build_parser()
        args = parser.parse_args(
            ["set-id", "--port", "/dev/ttyACM0", "--from", "1", "--to", "3"]
        )
        assert args.from_id == 1
        assert args.to_id == 3

    def test_scan_defaults(self):
        parser = cs.build_parser()
        args = parser.parse_args(["scan"])
        assert args.port == "/dev/ttyACM0"
        assert args.scan_max == cs.ID_MAX

    def test_missing_command_errors(self):
        parser = cs.build_parser()
        with pytest.raises(SystemExit):
            parser.parse_args([])
