import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "RUN" / "GA" / "plot_formation_fov_multistyle_suite.py"
SOURCE = (
    ROOT
    / "RUN"
    / "GA"
    / "dynamic_topology"
    / "figures"
    / "source"
    / "formation_fov_multistyle_suite_v5_seed41.json"
)


def load_renderer_module():
    spec = importlib.util.spec_from_file_location("multistyle_figure", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_v5_figure_source_is_bound_to_the_frozen_geometry_contract():
    data = json.loads(SOURCE.read_text(encoding="utf-8"))

    assert data["contractVersion"] == "formation-fov-multistyle-figure-source-v2"
    assert data["seed"] == 41
    assert data["snapshotTime"] == 80
    assert data["geometryTruthUsed"] is True
    assert data["posteriorUsed"] is False
    assert data["trackingResultUsed"] is False
    assert len(data["scenes"]) == 6

    for scene in data["scenes"]:
        assert scene["sceneGeometryVersion"] == "formation-fov-multistyle-v5"
        assert scene["fovTotalAngleDeg"] == 120
        assert scene["fovRange"] == 300
        assert scene["sensorsPerFormation"] == 6
        assert scene["trackingOutcomeAuthorized"] is False
        assert len(scene["sceneContractSha256"]) == 64
        assert len(scene["targetGroupIds"]) == scene["targetCount"]

        is_stress = scene["sceneStyle"] == "orthogonal-crossing"
        assert scene["formalValidationAuthorized"] is (not is_stress)
        expected_status = (
            "stress-only-v5"
            if is_stress
            else "held-out-geometry-gate-frozen-v5"
        )
        assert scene["sceneCalibrationStatus"] == expected_status


def test_renderer_rejects_within_formation_heading_drift(tmp_path):
    data = json.loads(SOURCE.read_text(encoding="utf-8"))
    data["scenes"][0]["sensorHeadingRad"][1] += 0.01
    forged = tmp_path / "forged.json"
    forged.write_text(json.dumps(data), encoding="utf-8")

    renderer = load_renderer_module()
    with pytest.raises(ValueError, match="share a heading"):
        renderer.load_source(forged)


def test_renderer_rejects_a_well_formed_but_unregistered_scene_digest(tmp_path):
    data = json.loads(SOURCE.read_text(encoding="utf-8"))
    data["scenes"][0]["sceneContractSha256"] = "0" * 64
    forged = tmp_path / "forged-digest.json"
    forged.write_text(json.dumps(data), encoding="utf-8")

    renderer = load_renderer_module()
    with pytest.raises(ValueError, match="registered scene digest"):
        renderer.load_source(forged)


def test_displayed_x36_sensor_nodes_and_fov_outlines_are_inside_frames():
    renderer = load_renderer_module()
    data = renderer.load_source(SOURCE)

    for scene in data["scenes"][:3]:
        x_limits, y_limits = renderer.scene_plot_bounds([scene])
        snapshot = scene["snapshotTime"] - 1
        sensor_x = renderer.as_float_array(scene["sensorX"])
        sensor_y = renderer.as_float_array(scene["sensorY"])
        group_ids = np.asarray(scene["sensorGroupIds"], dtype=int)
        headings = np.asarray(scene["sensorHeadingRad"], dtype=float)
        half_angle = np.radians(scene["fovHalfAngleDeg"])

        assert np.min(sensor_x[:, snapshot]) > x_limits[0]
        assert np.max(sensor_x[:, snapshot]) < x_limits[1]
        assert np.min(sensor_y[:, snapshot]) > y_limits[0]
        assert np.max(sensor_y[:, snapshot]) < y_limits[1]

        for group_id in np.unique(group_ids):
            sensor_idx = int(np.flatnonzero(group_ids == group_id)[0])
            angles = np.linspace(
                headings[sensor_idx] - half_angle,
                headings[sensor_idx] + half_angle,
                121,
            )
            x = sensor_x[sensor_idx, snapshot] + scene["fovRange"] * np.cos(
                angles
            )
            y = sensor_y[sensor_idx, snapshot] + scene["fovRange"] * np.sin(
                angles
            )
            assert np.min(x) > x_limits[0]
            assert np.max(x) < x_limits[1]
            assert np.min(y) > y_limits[0]
            assert np.max(y) < y_limits[1]


def test_python_renderer_creates_editable_vector_and_preview_outputs(tmp_path):
    output = tmp_path / "formation_fov_multistyle_suite_v2"
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--data",
            str(SOURCE),
            "--output",
            str(output),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr

    svg = output.with_suffix(".svg")
    pdf = output.with_suffix(".pdf")
    png = output.with_suffix(".png")
    assert svg.stat().st_size > 20_000
    assert pdf.stat().st_size > 10_000
    assert png.stat().st_size > 50_000

    svg_text = svg.read_text(encoding="utf-8")
    assert "<image" not in svg_text
    assert "Offset-corridor convoy" in svg_text
    assert "Orthogonal crossing" in svg_text
    assert "Linear relay" in svg_text
    assert "518.4pt" in svg_text
    assert "345.6pt" in svg_text


def test_vector_outputs_are_byte_reproducible(tmp_path):
    first = tmp_path / "first"
    second = tmp_path / "second"
    for output in (first, second):
        result = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                "--data",
                str(SOURCE),
                "--output",
                str(output),
            ],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr

    assert first.with_suffix(".svg").read_bytes() == second.with_suffix(
        ".svg"
    ).read_bytes()
    assert first.with_suffix(".pdf").read_bytes() == second.with_suffix(
        ".pdf"
    ).read_bytes()
