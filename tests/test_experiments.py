"""Tests for jobs, sweeps, the runner, the store and the analysis pipeline.

Two of these are regressions for bugs that produced *plausible but wrong* output rather
than a crash, which is the dangerous kind: an empty store that reported success, and a
parallel chunking bug that evaluated every job against the first job's hand and object.
"""

from __future__ import annotations

import numpy as np
import pytest

from morpholab.analysis import (
    ablation, bootstrap_ci, cliffs_delta, cohens_d, compare_groups, failure_breakdown,
    holm_correction, load_results, pareto_front, paired_compare, required_n, summarize_by,
    to_markdown, wilson_interval,
)
from morpholab.experiments import Job, ResultStore, RunnerOptions, SweepSpec, expand, run_jobs

pytest.importorskip("mujoco")
pd = pytest.importorskip("pandas")


# --------------------------------------------------------------------------------------
# jobs and sweeps
# --------------------------------------------------------------------------------------

def test_job_id_depends_on_content_not_on_labels():
    a = Job(morphology="birh_v1", obj="sphere_medium", task="lift")
    assert a.job_id == Job(morphology="birh_v1", obj="sphere_medium", task="lift").job_id
    assert a.job_id != a.with_trial(1).job_id
    # Tags label a job; they do not change what it computes.
    tagged = Job(morphology="birh_v1", obj="sphere_medium", task="lift", tags={"arm": "A"})
    assert tagged.job_id == a.job_id


def test_jobs_sharing_a_scene_share_a_group_key():
    a = Job(morphology="birh_v1", obj="sphere_medium", task="lift", trial=0)
    b = Job(morphology="birh_v1", obj="sphere_medium", task="lift", trial=5)
    c = Job(morphology="birh_v1", obj="cube_medium", task="lift")
    assert a.group_key == b.group_key != c.group_key


def test_job_seeds_are_derived_from_content():
    a = Job(morphology="birh_v1", obj="sphere_medium", task="lift", trial=3)
    assert a.rng().entropy == Job.from_dict(a.to_dict()).rng().entropy
    assert a.rng().entropy != a.with_trial(4).rng().entropy


def test_sweep_expands_object_sets_and_preset_tags():
    jobs = expand(SweepSpec(morphologies=["tag:birh"], objects=["quick"], tasks=["lift"]))
    hands = {j.morphology for j in jobs}
    assert "birh_v1" in hands and "birh_v2" in hands
    assert {j.obj for j in jobs} == {"sphere_medium", "cube_medium", "cylinder_upright", "marker"}


def test_sweep_respects_object_task_affordances():
    """Scoring a hand on pinching a bottle measures the pairing, not the hand."""
    filtered = expand(SweepSpec(morphologies=["birh_v1"], objects=["core15"], tasks=["pinch"]))
    unfiltered = expand(SweepSpec(morphologies=["birh_v1"], objects=["core15"], tasks=["pinch"],
                                  respect_object_filter=False))
    assert 0 < len(filtered) < len(unfiltered)


def test_sweep_rejects_unknown_names():
    with pytest.raises(KeyError, match="neither an object nor an object set"):
        expand(SweepSpec(morphologies=["birh_v1"], objects=["not_a_thing"]))


# --------------------------------------------------------------------------------------
# store
# --------------------------------------------------------------------------------------

def test_store_round_trips_and_tracks_completed_ids(tmp_path):
    store = ResultStore(tmp_path / "results.jsonl")
    store.append({"job_id": "a", "success": True, "value": np.float64(1.5)})
    store.append({"job_id": "b", "success": False, "arr": np.array([1, 2])})
    assert store.completed_ids() == {"a", "b"}
    assert len(store) == 2
    assert store.rows()[0]["value"] == 1.5
    assert store.rows()[1]["arr"] == [1, 2]


def test_empty_store_is_truthy(tmp_path):
    """Regression: ``__len__`` made an empty store falsy, so ``if store:`` silently
    discarded it and a completed sweep wrote nothing."""
    store = ResultStore(tmp_path / "results.jsonl")
    assert len(store) == 0
    assert bool(store) is True


def test_store_skips_a_truncated_final_line(tmp_path):
    """A process killed mid-write must cost one job, not the whole file."""
    path = tmp_path / "results.jsonl"
    path.write_text('{"job_id":"a","success":true}\n{"job_id":"b","suc')
    store = ResultStore(path)
    assert store.completed_ids() == {"a"}


def test_store_persists_and_reloads_morphologies(tmp_path):
    from morpholab.morphology import get_preset

    store = ResultStore(tmp_path / "results.jsonl")
    spec = get_preset("birh_v1")
    store.save_morphology(spec)
    assert store.load_morphology(spec.morphology_id).morphology_id == spec.morphology_id


# --------------------------------------------------------------------------------------
# runner
# --------------------------------------------------------------------------------------

@pytest.fixture(scope="module")
def small_run(tmp_path_factory):
    root = tmp_path_factory.mktemp("run")
    store = ResultStore(root / "results.jsonl")
    jobs = expand(SweepSpec(
        experiment="test", morphologies=["birh_v1", "two_finger"],
        objects=["sphere_medium", "cube_medium"], tasks=["lift"], trials=2,
    ))
    run_jobs(jobs, store=store, options=RunnerOptions(workers=1, progress=False))
    return store


def test_each_job_is_evaluated_against_its_own_hand_and_object(small_run):
    """Regression: chunking flattened distinct scene groups into one ``execute_group``
    call, so every job in a chunk was silently evaluated against the first job's scene --
    a full, plausible, entirely wrong result set."""
    df = small_run.to_dataframe()
    assert set(df["hand"]) == {"birh_v1", "two_finger"}
    assert set(df["object"]) == {"sphere_medium", "cube_medium"}
    assert len(df) == 8
    assert df.groupby(["hand", "object"]).size().eq(2).all()


def test_results_are_independent_of_worker_count(tmp_path):
    jobs = expand(SweepSpec(experiment="t", morphologies=["birh_v1"],
                            objects=["sphere_medium", "cube_medium"], tasks=["lift"]))
    outputs = {}
    for workers in (1, 2):
        store = ResultStore(tmp_path / f"w{workers}.jsonl")
        run_jobs(jobs, store=store, options=RunnerOptions(workers=workers, progress=False))
        outputs[workers] = {
            r["job_id"]: (r["success"], round(r["grasp_epsilon"], 9)) for r in store
        }
    assert outputs[1] == outputs[2]


def test_resume_skips_completed_jobs(small_run):
    jobs = expand(SweepSpec(
        experiment="test", morphologies=["birh_v1", "two_finger"],
        objects=["sphere_medium", "cube_medium"], tasks=["lift"], trials=2,
    ))
    again = run_jobs(jobs, store=small_run, options=RunnerOptions(workers=1, progress=False))
    assert again == []
    assert len(small_run) == 8


def test_a_broken_job_records_an_error_row_without_killing_the_sweep(tmp_path):
    store = ResultStore(tmp_path / "r.jsonl")
    jobs = [
        Job(morphology="birh_v1", obj="sphere_medium", task="lift"),
        Job(morphology="no_such_hand", obj="sphere_medium", task="lift"),
    ]
    run_jobs(jobs, store=store, options=RunnerOptions(workers=1, progress=False))
    rows = {r.get("hand"): r for r in store}
    assert rows["birh_v1"]["success"] in (True, False)
    assert rows["no_such_hand"]["outcome"] == "error"
    assert "cannot resolve morphology" in rows["no_such_hand"]["error"]


# --------------------------------------------------------------------------------------
# statistics
# --------------------------------------------------------------------------------------

def test_wilson_interval_stays_inside_zero_one():
    """The reason the normal approximation is never used: at n=30, p=0.97 it exceeds 1."""
    assert wilson_interval(30, 30).high <= 1.0
    assert wilson_interval(0, 30).low >= 0.0
    assert wilson_interval(29, 30).high < 1.0


def test_wilson_interval_narrows_with_n():
    assert wilson_interval(15, 30).half_width > wilson_interval(150, 300).half_width


def test_bootstrap_interval_covers_the_mean():
    rng = np.random.default_rng(0)
    x = rng.normal(2.0, 1.0, 200)
    ci = bootstrap_ci(x, n_boot=2000)
    assert ci.low < 2.0 < ci.high


def test_clustered_bootstrap_is_wider_than_the_naive_one():
    """Trials within an object share its difficulty; ignoring the clustering understates
    the interval."""
    rng = np.random.default_rng(1)
    offsets = rng.normal(0, 1.0, 8)
    values = np.concatenate([offsets[i] + rng.normal(0, 0.1, 25) for i in range(8)])
    clusters = np.repeat(np.arange(8), 25)
    naive = bootstrap_ci(values, n_boot=2000)
    clustered = bootstrap_ci(values, clusters=clusters, n_boot=2000)
    assert clustered.half_width > naive.half_width


def test_effect_sizes_have_the_expected_sign_and_scale():
    a, b = np.array([1.0, 1.1, 0.9, 1.05]), np.array([0.0, 0.1, -0.1, 0.05])
    assert cohens_d(a, b) > 2.0
    assert cliffs_delta(a, b) == pytest.approx(1.0)
    assert cliffs_delta(b, a) == pytest.approx(-1.0)


def test_paired_test_is_more_powerful_than_the_unpaired_one():
    """The reason ablations pair on (object, task, trial)."""
    rng = np.random.default_rng(2)
    base = rng.normal(0, 3.0, 40)
    a, b = base, base - 0.5
    assert paired_compare(a, b).p_mannwhitney < compare_groups(a, b).p_mannwhitney


def test_holm_correction_is_monotone_and_conservative():
    raw = [compare_groups(np.random.default_rng(i).normal(0, 1, 30),
                          np.random.default_rng(100 + i).normal(0.1 * i, 1, 30))
           for i in range(6)]
    adjusted = holm_correction(raw)
    for r, a in zip(raw, adjusted):
        assert a.p_adjusted >= r.p_mannwhitney - 1e-12
    ordered = sorted(adjusted, key=lambda c: c.p_mannwhitney)
    assert all(x.p_adjusted <= y.p_adjusted + 1e-12 for x, y in zip(ordered, ordered[1:]))


def test_power_analysis_matches_the_textbook_value():
    assert required_n(0.1, baseline=0.5) == pytest.approx(393, abs=2)
    assert required_n(0.2, baseline=0.5) < required_n(0.1, baseline=0.5)


# --------------------------------------------------------------------------------------
# summaries
# --------------------------------------------------------------------------------------

def test_summarize_reports_intervals_per_group(small_run):
    table = summarize_by(small_run.to_dataframe(), "hand")
    assert set(table["hand"]) == {"birh_v1", "two_finger"}
    assert (table["success_lo"] <= table["success"]).all()
    assert (table["success"] <= table["success_hi"]).all()


def test_failure_breakdown_rows_sum_to_one(small_run):
    table = failure_breakdown(small_run.to_dataframe())
    assert np.allclose(table.sum(axis=1), 1.0)


def test_pareto_front_is_non_dominated(small_run):
    front = pareto_front(small_run.to_dataframe())
    costs = front["complexity"].to_numpy()
    scores = front["success"].to_numpy()
    assert (np.diff(costs) > 0).all()
    assert (np.diff(scores) > 0).all()


def test_ablation_pairs_on_shared_cells(small_run):
    result = ablation(small_run.to_dataframe(), factor="hand",
                      baseline="birh_v1", variant="two_finger")
    assert result.paired
    assert result.n_a == result.n_b > 0


def test_markdown_needs_no_optional_dependency(small_run):
    text = to_markdown(summarize_by(small_run.to_dataframe(), "hand").set_index("hand"))
    assert text.startswith("| hand") and "|---" in text
