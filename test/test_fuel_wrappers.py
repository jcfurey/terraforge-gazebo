# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for tree_processor.missing_fuel_wrappers.

The Fuel-mode preflight check walks GZ_SIM_RESOURCE_PATH plus any
caller-provided roots looking for `tree_fuel_<i>/model.sdf`. Verify it
correctly reports missing vs present wrappers under different root
configurations.
"""

import os

import pytest

np_skip = pytest.importorskip('numpy')  # noqa: F841 - tree_processor imports numpy at module load
shapely_skip = pytest.importorskip('shapely')  # noqa: F841

from terraforge.data_processing.tree_processor import (  # noqa: E402
    TREE_VARIANTS,
    fuel_wrapper_model_names,
    missing_fuel_wrappers,
)


def _make_wrappers(root, names):
    for name in names:
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'model.sdf'), 'w') as f:
            f.write('<sdf/>')


def test_all_missing_when_no_roots(tmp_path, monkeypatch):
    monkeypatch.delenv('GZ_SIM_RESOURCE_PATH', raising=False)
    assert missing_fuel_wrappers() == fuel_wrapper_model_names()


def test_all_present_from_extra_roots(tmp_path, monkeypatch):
    monkeypatch.delenv('GZ_SIM_RESOURCE_PATH', raising=False)
    _make_wrappers(tmp_path, fuel_wrapper_model_names())
    assert missing_fuel_wrappers(extra_roots=[str(tmp_path)]) == []


def test_partial_present(tmp_path, monkeypatch):
    monkeypatch.delenv('GZ_SIM_RESOURCE_PATH', raising=False)
    _make_wrappers(tmp_path, ['tree_fuel_0', 'tree_fuel_2'])
    missing = missing_fuel_wrappers(extra_roots=[str(tmp_path)])
    assert 'tree_fuel_0' not in missing
    assert 'tree_fuel_2' not in missing
    assert set(missing) == {'tree_fuel_1', 'tree_fuel_3', 'tree_fuel_4'}


def test_gz_sim_resource_path_is_searched(tmp_path, monkeypatch):
    _make_wrappers(tmp_path, fuel_wrapper_model_names())
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', str(tmp_path))
    assert missing_fuel_wrappers() == []


def test_gz_sim_resource_path_colon_separated(tmp_path, monkeypatch):
    root_a = tmp_path / 'a'
    root_b = tmp_path / 'b'
    root_a.mkdir()
    root_b.mkdir()
    _make_wrappers(root_a, ['tree_fuel_0', 'tree_fuel_1'])
    _make_wrappers(root_b, ['tree_fuel_2', 'tree_fuel_3', 'tree_fuel_4'])
    sep = ';' if os.name == 'nt' else ':'
    monkeypatch.setenv('GZ_SIM_RESOURCE_PATH', f'{root_a}{sep}{root_b}')
    assert missing_fuel_wrappers() == []


def test_wrapper_count_matches_tree_variants():
    assert len(fuel_wrapper_model_names()) == TREE_VARIANTS
