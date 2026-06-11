# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Regression test: ``python3 -m terraforge`` must work.

setup.cfg routes the console_script wrappers into lib/terraforge_gazebo/
for `ros2 run`, which means NO pip install scenario (standalone, venv,
editable) ever puts a bare `terraforge` command on PATH. The module
entry point is therefore the documented standalone invocation; this
pins its existence.
"""
import subprocess
import sys

import pytest

pytest.importorskip('click')


def test_python_dash_m_terraforge_help():
    result = subprocess.run(
        [sys.executable, '-m', 'terraforge', '--help'],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stderr
    assert 'generate-world' in result.stdout
    assert 'list-tile-providers' in result.stdout
