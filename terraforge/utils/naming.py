# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Identifier sanitation for values that flow into paths or SDF XML.

`world_name` ends up in filesystem paths (per-world media subdir) and as
absolute `file://` URIs baked into the generated world. `performer_ref`
ends up in SDF `<model name>`, `<performer name>`, and `<ref>` elements.
Letting either carry `..`, `/`, XML metacharacters, whitespace, or
control characters produces path traversal or invalid SDF.
"""

import re


_SAFE_IDENT = re.compile(r'^[A-Za-z0-9_]([A-Za-z0-9_.\-]*[A-Za-z0-9_])?$')


def safe_identifier(value: str, *, field: str) -> str:
    """Return `value` unchanged if it's a safe filesystem/SDF identifier.

    Raises ``ValueError`` otherwise. The allowed alphabet is
    ``[A-Za-z0-9_.-]`` with the first AND last characters restricted to
    ``[A-Za-z0-9_]`` — disallows leading dot/dash (looks like a shell
    flag or a hidden file) and trailing dot/dash (looks like a file
    extension stub or a hyphenated suffix).
    """
    if not isinstance(value, str) or not value:
        raise ValueError(f'{field} must be a non-empty string')
    if not _SAFE_IDENT.match(value):
        raise ValueError(
            f"{field}={value!r} contains disallowed characters. "
            f"Allowed: letters, digits, '_', '.', '-'; must start AND end "
            f"with a letter/digit/underscore."
        )
    return value
