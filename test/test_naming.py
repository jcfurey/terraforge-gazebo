# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for terraforge.utils.naming.safe_identifier.

Guards the identifier whitelist used for --world-name and
--performer-ref, which flow into filesystem paths and SDF XML
respectively.
"""

import pytest

from terraforge.utils.naming import safe_identifier


@pytest.mark.parametrize('value', [
    'generated_world',
    'rovermax',
    'My-World.v2',
    'W123_test',
    '0_leading_digit',
    '_leading_underscore',
])
def test_accepts_valid_identifiers(value):
    assert safe_identifier(value, field='x') == value


@pytest.mark.parametrize('value', [
    '',
    '../evil',
    'foo/bar',
    'foo\\bar',
    'foo bar',
    'foo;rm -rf /',
    '.hidden',
    '-flag',
    'trailing_dot.',
    'trailing_dash-',
    'a<b',
    'a>b',
    'a&b',
    'a"b',
    "a'b",
    'a\nb',
])
def test_rejects_unsafe_identifiers(value):
    with pytest.raises(ValueError):
        safe_identifier(value, field='x')


@pytest.mark.parametrize('value', [None, 42, b'bytes', 3.14, object()])
def test_rejects_non_strings(value):
    with pytest.raises(ValueError):
        safe_identifier(value, field='x')
