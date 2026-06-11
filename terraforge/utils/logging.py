# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
import logging
import sys


def setup_logger(name, log_level=logging.INFO):
    """Return a logger instance configured to write to stdout.

    Idempotent: re-calling with the same ``name`` won't add a second
    handler, so log lines never double-print. Without this guard, an
    accidental second call (test fixture re-imports, REPL reloads, GUI
    resetup, etc.) silently doubles every log line through the new
    duplicate handler.
    """
    logger = logging.getLogger(name)
    logger.setLevel(log_level)
    # Every logger configured here gets its own stdout handler, so stop
    # propagation to ancestors: 'terraforge.gui' is a dotted child of
    # 'terraforge' (the GUI imports cli, so both exist with handlers),
    # and without this every GUI log line printed twice. Also keeps a
    # host application's root-logger config from re-emitting our lines.
    logger.propagate = False

    if not logger.handlers:
        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setLevel(log_level)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        stream_handler.setFormatter(formatter)
        logger.addHandler(stream_handler)
    return logger


logger = setup_logger('gazebomg')
