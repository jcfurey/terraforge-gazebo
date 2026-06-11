# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Module entry point: ``python3 -m terraforge``.

The console_script wrappers declared in setup.py are redirected by
setup.cfg's ``[install] install_scripts`` to ``lib/terraforge_gazebo/``
so that ``ros2 run terraforge_gazebo terraforge`` works inside a colcon
workspace. The side effect is that a plain ``pip install`` (standalone,
venv, or editable) never puts ``terraforge`` on PATH — the redirect
applies there too. ``python3 -m terraforge`` works in every install
scenario, so it is the documented standalone invocation.
"""
from terraforge.cli import cli

if __name__ == '__main__':
    cli()
