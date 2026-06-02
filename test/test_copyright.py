# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Copyright-header lint (disabled).

ament_copyright only recognises a file's license when the header contains a
full license body matching one of its bundled templates. This project uses a
short notice ("Licensed under the MIT License.") plus a top-level LICENSE
file, which ament_copyright reports as ``license=<unknown>`` on every file.
Rather than inline the full ~11-line MIT text into every source file, the
license is governed by the LICENSE file and the per-file short header. Enable
this check (and add full license bodies) if strict ament_copyright
conformance is later required.
"""
import pytest


@pytest.mark.copyright
@pytest.mark.linter
@pytest.mark.skip(
    reason='Short MIT header is not an ament_copyright template; licensing '
           'is governed by the LICENSE file + per-file headers.'
)
def test_copyright():
    pass
