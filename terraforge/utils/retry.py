"""Small exponential-backoff retry helper for transient network failures.

Used by DEM (SRTM / Overpass), OSM (Overpass via osmnx), and satellite
tile downloads. Intentionally tiny — no third-party tenacity/backoff
dependency, just enough to absorb a 429 or a brief TCP reset without
aborting a 500-tile mosaic.
"""

import time

from terraforge.utils.logging import logger


def retry_call(
    fn,
    *,
    attempts=3,
    initial_delay=1.0,
    backoff=2.0,
    exceptions=(Exception,),
    label='operation',
):
    """Call ``fn()`` with exponential backoff.

    Retries up to ``attempts`` times, sleeping ``initial_delay *
    backoff**i`` seconds between attempts. Re-raises the final
    exception if all attempts fail. Logs a warning on each retry so
    operators can see transient failures in the run log.
    """
    if attempts < 1:
        raise ValueError("attempts must be >= 1")
    last_exc = None
    delay = initial_delay
    for i in range(1, attempts + 1):
        try:
            return fn()
        except exceptions as e:
            last_exc = e
            if i == attempts:
                break
            logger.warning(
                f"{label}: attempt {i}/{attempts} failed ({type(e).__name__}: {e}); "
                f"retrying in {delay:.1f}s"
            )
            time.sleep(delay)
            delay *= backoff
    raise last_exc


def scrub_key(url: str, api_key: str) -> str:
    """Return ``url`` with ``api_key`` substring replaced by ``<redacted>``.

    For logging tile URLs. Not a security boundary (the key is still in
    memory and can be logged by other libraries), just a best-effort
    hygiene step so our own log lines don't leak tokens.
    """
    if not api_key:
        return url
    return url.replace(api_key, '<redacted>')
