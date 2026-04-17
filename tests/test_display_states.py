"""
"""Tests for the interactive_planning script with the PyHPP (pyhpp) backend.

These tests cover:
- Feasible goal-state enumeration from the spacelab config (no HPP required).
- The ``--backend pyhpp --print-goals`` CLI path (no HPP connection required).
- DisplayStatesTask instantiation with backend="pyhpp".
- New B1-B8 backend methods exercised through the task's planner attribute.
"""

from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path

import pytest

# ---------------------------------------------------------------------------
# Ensure the script config directory is on sys.path so spacelab_config loads.
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).parents[1] / "script"
CONFIG_DIR = SCRIPT_DIR / "config"

if str(CONFIG_DIR) not in sys.path:
    sys.path.insert(0, str(CONFIG_DIR))

# ---------------------------------------------------------------------------
# Import the task script as a module (adds config dir to its own sys.path).
# ---------------------------------------------------------------------------
TASK_SCRIPT = SCRIPT_DIR / "spacelab" / "interactive_planning.py"

_spec = importlib.util.spec_from_file_location(
    "interactive_planning", str(TASK_SCRIPT)
)
_task_mod: types.ModuleType | None = None
_import_error: Exception | None = None

try:
    _task_mod = importlib.util.module_from_spec(_spec)  # type: ignore[arg-type]
    _spec.loader.exec_module(_task_mod)  # type: ignore[union-attr]
except Exception as exc:
    _import_error = exc


requires_task_script = pytest.mark.skipif(
    _task_mod is None,
    reason=f"interactive_planning could not be imported: {_import_error}",
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _load_cfg():
    """Return the DisplayAllStates config (no HPP needed)."""
    cfg_mod = importlib.import_module("spacelab_config")
    return cfg_mod.TaskConfigurations.DisplayAllStates


# ---------------------------------------------------------------------------
# Config-only tests (no HPP process, no pyhpp module needed)
# ---------------------------------------------------------------------------


class TestFeasibleGoalStates:
    """Verify config-level goal-state enumeration works for pyhpp backend."""

    def test_feasible_grasp_goal_states_nonempty(self):
        """feasible_grasp_goal_states() returns at least one state."""
        cfg = _load_cfg()
        goals = cfg.feasible_grasp_goal_states()
        assert len(goals) > 0, "Expected at least one feasible grasp goal state"

    def test_all_goal_states_follow_naming_convention(self):
        """Every goal state matches '<gripper> grasps <handle>'."""
        cfg = _load_cfg()
        for state in cfg.feasible_grasp_goal_states():
            assert " grasps " in state, (
                f"Goal state does not follow '<gripper> grasps <handle>': {state!r}"
            )

    def test_valid_pairs_non_empty(self):
        """VALID_PAIRS in config has at least one gripper-handle mapping."""
        cfg = _load_cfg()
        assert cfg.VALID_PAIRS, "VALID_PAIRS should not be empty"

    def test_objects_non_empty(self):
        """Config contains at least one object."""
        cfg = _load_cfg()
        assert cfg.OBJECTS, "OBJECTS should not be empty"

    def test_grippers_non_empty(self):
        """Config contains at least one gripper."""
        cfg = _load_cfg()
        assert cfg.GRIPPERS, "GRIPPERS should not be empty"


# ---------------------------------------------------------------------------
# CLI --print-goals path (no HPP needed)
# ---------------------------------------------------------------------------


@requires_task_script
class TestMainPrintGoals:
    """Smoke tests for ``main(["--backend", "pyhpp", "--print-goals"])``."""

    def test_print_goals_pyhpp_returns_zero(self, capsys):
        """main returns 0 and prints at least one state line."""
        rc = _task_mod.main(["--backend", "pyhpp", "--print-goals"])
        assert rc == 0

        captured = capsys.readouterr()
        lines = [ln.strip() for ln in captured.out.splitlines() if ln.strip()]
        assert len(lines) > 0, "Expected printed goal states on stdout"
        assert all(" grasps " in ln for ln in lines), (
            "All output lines should be '<gripper> grasps <handle>'"
        )

    def test_print_goals_corba_and_pyhpp_produce_same_states(self, capsys):
        """Both backends enumerate the same goal states (config-driven)."""
        _task_mod.main(["--backend", "pyhpp", "--print-goals"])
        pyhpp_out = capsys.readouterr().out.strip().splitlines()

        _task_mod.main(["--backend", "corba", "--print-goals"])
        corba_out = capsys.readouterr().out.strip().splitlines()

        assert sorted(pyhpp_out) == sorted(corba_out), (
            "CORBA and PyHPP backends should produce the same goal states"
        )

    def test_print_goals_with_limit(self, capsys):
        """--limit restricts the number of printed goal states."""
        rc = _task_mod.main(
            ["--backend", "pyhpp", "--print-goals", "--limit", "2"]
        )
        assert rc == 0
        lines = [ln for ln in capsys.readouterr().out.splitlines() if ln.strip()]
        assert len(lines) <= 2


# ---------------------------------------------------------------------------
# DisplayStatesTask instantiation with pyhpp backend
# ---------------------------------------------------------------------------


try:
    from agimus_spacelab.backends.pyhpp import HAS_PYHPP
except Exception:
    HAS_PYHPP = False

requires_pyhpp = pytest.mark.skipif(
    not HAS_PYHPP, reason="PyHPP (hpp-python) not available"
)


@requires_task_script
class TestDisplayStatesTaskInit:
    """Check DisplayStatesTask(backend='pyhpp') creates the right objects."""

    def test_instantiation_pyhpp(self):
        """DisplayStatesTask can be created with backend pyhpp."""
        task = _task_mod.DisplayStatesTask(backend="pyhpp")
        assert task.backend == "pyhpp"
        assert task.use_factory is True
        assert task.task_config is not None

    def test_instantiation_stores_empty_pyhpp_constraints(self):
        """pyhpp_constraints starts as empty dict."""
        task = _task_mod.DisplayStatesTask(backend="pyhpp")
        assert task.pyhpp_constraints == {}

    @requires_pyhpp
    def test_planner_is_pyhpp_backend(self):
        """After instantiation, calling planner would use PyHPPBackend."""
        from agimus_spacelab.backends.pyhpp import PyHPPBackend

        task = _task_mod.DisplayStatesTask(backend="pyhpp")
        # planner is None until setup() is called, but scene_builder backend
        # should be "pyhpp"
        assert task.scene_builder.backend == "pyhpp"
        assert task.planner is None  # not set up yet


# ---------------------------------------------------------------------------
# Backend method coverage via mock (B1-B8)
# ---------------------------------------------------------------------------


@requires_pyhpp
class TestB1ToB8WithMockSetup:
    """Exercise B1-B8 methods on a planner instance (no HPP server needed)."""

    @pytest.fixture
    def planner(self):
        from agimus_spacelab.backends.pyhpp import PyHPPBackend

        return PyHPPBackend()

    def test_configure_transition_planner_inner_planner_type(self, planner):
        planner.configure_transition_planner(inner_planner_type="DiffusingPlanner")
        assert planner._transition_inner_planner_type == "DiffusingPlanner"

    def test_set_path_projection_toggle(self, planner):
        planner.set_path_projection(False)
        assert planner._use_progressive_projector is False
        planner.set_path_projection(True)
        assert planner._use_progressive_projector is True

    def test_clear_stored_paths(self, planner):
        planner._stored_paths.append(object())
        planner._stored_paths.append(object())
        n = planner.clear_stored_paths(verbose=False)
        assert n == 2
        assert planner._stored_paths == []

    def test_resolve_transition_int(self, planner):
        from unittest.mock import MagicMock

        tr = MagicMock()
        tr.name.return_value = "edge_0"
        mock_graph = MagicMock()
        mock_graph.getTransitions.return_value = [tr]
        planner.graph = mock_graph

        result = planner._resolve_transition(0)
        assert result is tr

    def test_play_path_vector_calls_store_and_play(self, planner):
        """play_path_vector stores the path, visualizes, then plays it."""
        from unittest.mock import MagicMock, patch, call

        fake_path = MagicMock()
        fake_path.length.return_value = 0.0

        with patch.object(planner, "store_path", return_value=0) as mock_store, \
             patch.object(planner, "visualize") as mock_viz, \
             patch.object(planner, "play_path") as mock_play:
            idx = planner.play_path_vector(fake_path)

        assert idx == 0
        mock_store.assert_called_once_with(fake_path)
        mock_viz.assert_called_once()
        mock_play.assert_called_once_with(0)

    def test_play_and_record_path_vector_returns_tuple(self, planner):
        """play_and_record_path_vector returns (path_index, video_file)."""
        from unittest.mock import MagicMock, patch

        fake_path = MagicMock()

        with patch.object(planner, "store_path", return_value=3) as mock_store, \
             patch.object(planner, "play_and_record_path", return_value="/tmp/out.mp4"):
            idx, video = planner.play_and_record_path_vector(fake_path)

        assert idx == 3
        assert video == "/tmp/out.mp4"
        mock_store.assert_called_once_with(fake_path)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
