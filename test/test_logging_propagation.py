# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Regression test: child loggers must not double-print via ancestors.

'terraforge.gui' is a dotted child of 'terraforge' in the logging
hierarchy and both get their own stdout handler from setup_logger (the
GUI imports cli, so both always coexist in GUI runs). Before propagation
was disabled, every GUI log line printed twice.
"""
import logging

from terraforge.utils.logging import setup_logger


def test_child_logger_does_not_propagate_to_parent_handler():
    parent = setup_logger('terraforge')
    child = setup_logger('terraforge.gui')

    class Counter(logging.Handler):
        def __init__(self):
            super().__init__()
            self.count = 0

        def emit(self, record):
            self.count += 1

    counter = Counter()
    parent.addHandler(counter)
    try:
        child.info('propagation probe')
    finally:
        parent.removeHandler(counter)
    assert counter.count == 0, (
        'child logger records must not reach ancestor handlers '
        '(GUI log lines printed twice before propagate=False)'
    )


def test_setup_logger_remains_idempotent():
    a = setup_logger('terraforge.idempotency_probe')
    n_handlers = len(a.handlers)
    b = setup_logger('terraforge.idempotency_probe')
    assert a is b
    assert len(b.handlers) == n_handlers
