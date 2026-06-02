# Copyright 2024 TerraForge Contributors
#
# Licensed under the MIT License.
"""Smoke tests for the run_generate_world cancellation + progress hooks.

Drives the public entry point with a cancel_flag that trips on the
first network stage and a progress_percent collector. Uses monkeypatch
to avoid actual network I/O or GDAL reprojection.
"""

import pytest

pytest.importorskip('click')
pytest.importorskip('osgeo')
pytest.importorskip('pyproj')

from terraforge import cli  # noqa: E402


def _never_call(*_a, **_kw):
    raise AssertionError('this stage should not have run')


def test_cancel_before_dem_download(tmp_path, monkeypatch):
    """An always-True cancel flag short-circuits before any network I/O.

    The pipeline bails at the first check, right after identifier
    validation and before any network I/O.
    """
    # Patch every stage that would otherwise run so the test doesn't
    # silently succeed by hitting a network path.
    monkeypatch.setattr(cli.elevation, 'download_dem', _never_call)
    monkeypatch.setattr(cli.osm, 'download_osm_buildings', _never_call)
    monkeypatch.setattr(cli.textures, 'download_satellite_texture_tiles', _never_call)

    with pytest.raises(InterruptedError):
        cli.run_generate_world(
            latitude=37.7749, longitude=-122.4194,
            radius=500.0, output_dir=str(tmp_path),
            world_name='test_cancel',
            cancel_flag=lambda: True,
        )


def test_progress_percent_is_called_in_order(tmp_path, monkeypatch):
    """Verify progress_percent fires monotonically at stage transitions.

    We monkeypatch download_dem to raise a sentinel after
    the first _pct(5) so we can inspect the call record without running
    the full pipeline.
    """
    percents = []

    class _Sentinel(Exception):
        pass

    def _raise(*_a, **_kw):
        raise _Sentinel()

    monkeypatch.setattr(cli.elevation, 'download_dem', _raise)

    with pytest.raises(_Sentinel):
        cli.run_generate_world(
            latitude=37.7749, longitude=-122.4194,
            radius=500.0, output_dir=str(tmp_path),
            world_name='test_progress',
            progress_percent=percents.append,
        )

    # The first stage marker (5 %, pre-DEM download) should have fired
    # before the sentinel blew up.
    assert percents == [5], percents
