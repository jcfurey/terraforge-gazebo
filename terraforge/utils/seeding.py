# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Stable per-process RNG seeding helper.

Used by tree_processor and building_processor for per-instance jitter
that needs to be reproducible across regens. Python's built-in ``hash()``
of strings (and tuples containing strings) is randomized per process —
``PYTHONHASHSEED`` defaults to a random value since Python 3.3 — so any
RNG keyed on ``hash(name)`` produces different results every run, even
with the same inputs.
"""

import hashlib


def stable_seed(*parts) -> int:
    """Return a deterministic 32-bit seed from string-castable ``parts``.

    The same inputs always produce the same seed, regardless of the
    process's PYTHONHASHSEED. Uses sha1 (truncated to 4 bytes) because
    it's stdlib, fast, and the seed quality matters far less than the
    determinism guarantee.
    """
    payload = '|'.join(str(p) for p in parts).encode('utf-8')
    return int.from_bytes(hashlib.sha1(payload).digest()[:4], 'big')
