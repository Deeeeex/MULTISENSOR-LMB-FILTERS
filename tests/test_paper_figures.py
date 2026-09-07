from pathlib import Path
import subprocess
import sys


def test_scalar_figure_data_contains_expected_sections():
    from docs.paper.figures.paper_figure_data import get_scalar_figure_data

    data = get_scalar_figure_data()
    assert "figure5" in data
    assert "figure6" in data
    assert data["figure5"]["arms"][-1] == "Balanced mode"
    assert data["figure6"]["consensus"]["labels"] == ["OSPA err.", "Loc. disag.", "Card. disp."]


def test_render_command_creates_pdf_outputs(tmp_path):
    from docs.paper.figures.render_paper_figures import render_all_figures

    outputs = render_all_figures(output_dir=tmp_path, include_figure4=False)

    assert (tmp_path / "figure5_factor_ablation.pdf").exists()
    assert (tmp_path / "figure6_ideal_support.pdf").exists()
    assert outputs["figure5"].suffix == ".pdf"
    assert outputs["figure6"].suffix == ".pdf"


def test_rendered_scalar_figures_are_nontrivial_pdfs(tmp_path):
    from docs.paper.figures.render_paper_figures import render_all_figures

    outputs = render_all_figures(output_dir=tmp_path, include_figure4=False)

    for key in ("figure5", "figure6"):
        path = outputs[key]
        assert path.exists()
        assert path.stat().st_size > 5_000


def test_figure_readme_mentions_expected_outputs():
    text = Path("docs/paper/figures/README.md").read_text()
    assert "figure4_main_ga_consensus.pdf" in text
    assert "figure3_existence_confidence_curve.pdf" in text
    assert "figure5_factor_ablation.pdf" in text
    assert "figure6_ideal_support.pdf" in text


def test_cli_render_command_creates_expected_outputs(tmp_path):
    script = Path("docs/paper/figures/render_paper_figures.py")
    result = subprocess.run(
        [sys.executable, str(script), "--output-dir", str(tmp_path)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert (tmp_path / "figure5_factor_ablation.pdf").exists()
    assert (tmp_path / "figure6_ideal_support.pdf").exists()


def test_load_and_render_figure4_from_csv(tmp_path):
    csv_path = tmp_path / "figure4_consensus_series.csv"
    csv_path.write_text(
        "\n".join(
            [
                "time,ospa_fixed,ospa_balanced,ospa_cardinality,rmse_fixed,rmse_balanced,rmse_cardinality,card_fixed,card_balanced,card_cardinality",
                "1,2.6,1.9,1.7,2.7,1.8,1.6,0.8,0.3,0.1",
                "2,2.5,1.8,1.6,2.6,1.7,1.5,0.7,0.28,0.09",
                "3,2.4,1.7,1.5,2.5,1.6,1.4,0.6,0.25,0.08",
            ]
        )
    )

    from docs.paper.figures.paper_figure_data import load_figure4_series
    from docs.paper.figures.render_paper_figures import render_all_figures

    series = load_figure4_series(csv_path)
    assert series["time"] == [1.0, 2.0, 3.0]

    outputs = render_all_figures(output_dir=tmp_path, include_figure4=True)
    assert (tmp_path / "figure4_main_ga_consensus.pdf").exists()
    assert outputs["figure4"].stat().st_size > 5_000


def test_renderer_does_not_use_internal_suptitles(tmp_path, monkeypatch):
    from matplotlib.figure import Figure
    from docs.paper.figures.render_paper_figures import render_all_figures

    original_suptitle = Figure.suptitle

    def fail_on_suptitle(self, *args, **kwargs):
        raise AssertionError(f"Unexpected suptitle call: args={args!r}")

    monkeypatch.setattr(Figure, "suptitle", fail_on_suptitle)

    csv_path = tmp_path / "figure4_consensus_series.csv"
    csv_path.write_text(
        "\n".join(
            [
                "time,ospa_fixed,ospa_balanced,ospa_cardinality,rmse_fixed,rmse_balanced,rmse_cardinality,card_fixed,card_balanced,card_cardinality",
                "1,2.6,1.9,1.7,2.7,1.8,1.6,0.8,0.3,0.1",
                "2,2.5,1.8,1.6,2.6,1.7,1.5,0.7,0.28,0.09",
                "3,2.4,1.7,1.5,2.5,1.6,1.4,0.6,0.25,0.08",
            ]
        )
    )

    try:
        render_all_figures(output_dir=tmp_path, include_figure4=True)
    finally:
        monkeypatch.setattr(Figure, "suptitle", original_suptitle)


def test_caption_registry_contains_all_main_figures():
    text = Path("docs/paper/figure_captions.md").read_text()
    for figure_no in range(1, 6):
        assert f"Figure {figure_no}" in text
    assert "Figure 6" not in text


def test_prompt_files_exist_for_figure1_and_figure2():
    figure1 = Path("docs/paper/figures/figure1_system_overview_prompt.md").read_text()
    figure2 = Path("docs/paper/figures/figure2_weight_factorization_prompt.md").read_text()

    assert "Prompt" in figure1
    assert "Required Elements" in figure1
    assert "Prompt" in figure2
    assert "Required Elements" in figure2


def test_render_all_figures_creates_figure3_pdf(tmp_path):
    from docs.paper.figures.render_paper_figures import render_all_figures

    outputs = render_all_figures(output_dir=tmp_path, include_figure4=False)

    figure3 = tmp_path / "figure3_existence_confidence_curve.pdf"
    assert figure3.exists()
    assert outputs["figure3"] == figure3
    assert figure3.stat().st_size > 5_000


def test_comm_level_fixed_balanced_data_matches_main_level_two():
    from docs.paper.figures.plot_comm_level_method_compare import load_rows

    csv_path = Path("docs/paper/figures/comm_level_fixed_balanced_n50.csv")
    rows = load_rows(csv_path)
    level_two = {
        (str(row["metric"]), str(row["method"])): round(float(row["mean"]), 3)
        for row in rows
        if int(row["level"]) == 2
    }
    assert level_two == {
        ("OSPA consensus error", "Fixed Metropolis"): 2.469,
        ("OSPA consensus error", "Balanced mode"): 1.779,
        ("Matched localization disagreement", "Fixed Metropolis"): 2.326,
        ("Matched localization disagreement", "Balanced mode"): 1.522,
        ("Cardinality dispersion", "Fixed Metropolis"): 0.716,
        ("Cardinality dispersion", "Balanced mode"): 0.188,
    }

    manuscript_results = Path(
        "docs/paper/els-cas-templates/sections/06_results.tex"
    ).read_text()
    assert (
        "2 & tiered link loss & $2.469 / 1.779$ & $2.326 / 1.522$ "
        "& $0.716 / 0.188$"
    ) in manuscript_results


def test_comm_level_two_method_renderer_creates_pdf_and_png(tmp_path):
    from docs.paper.figures.plot_comm_level_method_compare import load_rows, save_plot

    rows = load_rows(Path("docs/paper/figures/comm_level_fixed_balanced_n50.csv"))
    outputs = save_plot(rows, tmp_path / "comm_level_fixed_balanced_n50")

    assert {path.suffix for path in outputs} == {".pdf", ".png"}
    assert all(path.exists() and path.stat().st_size > 5_000 for path in outputs)
