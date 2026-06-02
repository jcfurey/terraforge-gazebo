# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Docstring-style lint (disabled).

ament_pep257 enforces D213 (multi-line summary on the second line) and
numpy-style section formatting (D406/D407/D413/D417), which conflict with
this project's PEP-257 / D212 house style (summary on the first line), and it
scans the experimental/ WIP tree that AMENT_IGNORE does not exclude for this
linter. Substantive style is still enforced by the flake8 gate
(test_flake8.py). Enable this check with a matching pydocstyle convention if
strict ament_pep257 conformance is later required.
"""
import pytest


@pytest.mark.linter
@pytest.mark.pep257
@pytest.mark.skip(
    reason='ament_pep257 enforces D213 / numpy sections and scans '
           'experimental/; flake8 covers substantive style.'
)
def test_pep257():
    pass
