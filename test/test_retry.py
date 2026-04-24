# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Unit tests for terraforge.utils.retry.retry_call / scrub_key."""

import pytest

from terraforge.utils.retry import retry_call, scrub_key


class _Counter:
    def __init__(self, fail_n, exc=RuntimeError('boom')):
        self.fail_n = fail_n
        self.calls = 0
        self.exc = exc

    def __call__(self):
        self.calls += 1
        if self.calls <= self.fail_n:
            raise self.exc
        return 'ok'


def test_succeeds_first_try():
    c = _Counter(fail_n=0)
    assert retry_call(c, attempts=3, initial_delay=0.0) == 'ok'
    assert c.calls == 1


def test_retries_then_succeeds():
    c = _Counter(fail_n=2)
    assert retry_call(c, attempts=3, initial_delay=0.0) == 'ok'
    assert c.calls == 3


def test_reraises_after_exhausting_attempts():
    c = _Counter(fail_n=10)
    with pytest.raises(RuntimeError, match='boom'):
        retry_call(c, attempts=3, initial_delay=0.0)
    assert c.calls == 3


def test_does_not_retry_on_unlisted_exception():
    c = _Counter(fail_n=1, exc=KeyError('nope'))
    with pytest.raises(KeyError):
        retry_call(c, attempts=3, initial_delay=0.0, exceptions=(ValueError,))
    assert c.calls == 1


def test_attempts_must_be_positive():
    with pytest.raises(ValueError):
        retry_call(lambda: 'x', attempts=0)


def test_scrub_key_redacts():
    url = 'https://example.com/tiles/1/2/3?key=SECRET123'
    assert 'SECRET123' not in scrub_key(url, 'SECRET123')
    assert '<redacted>' in scrub_key(url, 'SECRET123')


def test_scrub_key_empty_key_passthrough():
    url = 'https://example.com/tiles/1/2/3'
    assert scrub_key(url, '') == url
