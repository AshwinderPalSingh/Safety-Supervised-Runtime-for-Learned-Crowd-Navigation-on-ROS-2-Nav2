#!/usr/bin/env python3
"""Generates SarlAdapter's round-trip action-match fixture ( Phase 8 /
S4.7), by running the REAL reference predict() over many synthetic scenarios and mining the ones
where the top two candidates' values are genuinely close - not hand-picked. An obviously-best
action matches even with a wrong discount factor or reward term; a near-tied one doesn't.

For each scenario, captures: self_state (9 raw fields), each human's raw state (5 fields), and
the reference's own chosen action (vx, vy) - the C++ test rebuilds a WorldState from these,
runs it through this project's own SarlAdapter (candidate generation, one-step propagation,
rotation, real ONNX inference, reward+discount+argmax), and asserts its chosen velocity matches
the reference's chosen candidate exactly (candidates are a fixed discrete set, so an argmax
disagreement is a real reimplementation bug, not floating noise).

Run once, offline, against a checkout of tkkim-robot/Gazebo-CrowdNav (commit
9cad128d124f86bafe48d2cd11b5eee74bec77d9, matching il_model.pth - S1.8).

Usage: python3 generate_sarl_action_fixture.py /path/to/cloned/Gazebo-CrowdNav
"""
import configparser
import sys
from pathlib import Path


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} /path/to/cloned/Gazebo-CrowdNav", file=sys.stderr)
        return 1
    repo_path = sys.argv[1]
    sys.path.insert(0, repo_path)

    import numpy as np  # noqa: E402
    import torch  # noqa: E402
    from crowd_nav.policy.sarl import SARL  # noqa: E402
    from crowd_sim.envs.utils.state import FullState, ObservableState, JointState  # noqa: E402

    ckpt_dir = Path(repo_path) / "crowd_nav" / "data_sarl" / "output"
    policy_config = configparser.RawConfigParser()
    policy_config.read(str(ckpt_dir / "policy.config"))
    policy = SARL()
    policy.configure(policy_config)
    policy.get_model().load_state_dict(
        torch.load(str(ckpt_dir / "il_model.pth"), map_location="cpu"))
    policy.get_model().eval()
    policy.set_phase("test")
    policy.set_device(torch.device("cpu"))
    policy.time_step = 0.25
    policy.query_env = False  # deployment never has a training-time sim to query - S4.7

    def make_scenario(n_humans, seed):
        rng = np.random.RandomState(seed)
        # Keep the goal far enough from the start that reach_destination() never trips.
        self_state = FullState(
            rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5),
            0.3, rng.uniform(-3, 3) + 4.0, rng.uniform(-3, 3) + 4.0, 1.0, 0.0)
        humans = [
            ObservableState(
                rng.uniform(-3, 3), rng.uniform(-3, 3),
                rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), 0.3)
            for _ in range(n_humans)
        ]
        return self_state, humans

    scenarios = []
    for n_humans in (2, 3, 4, 5, 6):
        for seed in range(40):
            self_state, humans = make_scenario(n_humans, seed * 7 + n_humans * 101)
            js = JointState(self_state, humans)
            action = policy.predict(js)
            values = np.array(policy.action_values)
            sorted_idx = np.argsort(values)[::-1]
            top1, top2 = values[sorted_idx[0]], values[sorted_idx[1]]
            rel_gap = abs(top1 - top2) / max(abs(top1), 1e-9)
            # IMPORTANT (found while generating this fixture):
            # predict() does NOT see the raw `humans` list constructed above. This repo's own
            # crowd_sim/envs/utils/state.py ("CrowdNav State Modified Ver / add Dummy Ped ... /
            # FOV ROI Applied") applies a simulated depth-camera FOV filter
            # (JointState.fovFilter, 85.2deg x 12m matching a D435I) inside JointState.__init__,
            # appends one synthetic far-away "dummy" human along the agent's own heading, and
            # falls back to a different fixed dummy if literally nothing survives the filter -
            # THEN stores the result as js.human_states, which is what predict() actually
            # iterates. Capturing js.human_states (not the pre-filter `humans`) here is what
            # makes this fixture a fair test of the CANDIDATE-SEARCH reimplementation (rotation/
            # propagation/reward/discount/argmax) given the same humans, rather than an
            # apples-to-oranges comparison against a different, unfiltered human set - this
            # project's own SarlAdapter does not yet replicate the FOV filter itself (see
            # docs/phase8-findings.md for the follow-up this surfaced).
            scenarios.append({
                "self_state": self_state,
                "humans": list(js.human_states),
                "action": action,
                "rel_gap": rel_gap,
            })

    scenarios.sort(key=lambda s: s["rel_gap"])
    adversarial = scenarios[:10]
    typical = scenarios[-5:]

    out_path = Path(__file__).resolve().parent / "fixtures" / "sarl_action_reference.txt"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    def f64(x):
        # Plain Python float repr, not numpy's "np.float64(...)" repr (rng.uniform() and
        # anything derived from it are numpy scalars) - the C++ reader is a simple
        # whitespace/istream>>double parser and can't handle the numpy wrapper syntax.
        return repr(float(x))

    with open(out_path, "w") as f:
        f.write(
            "# source_repo=tkkim-robot/Gazebo-CrowdNav "
            "source_commit=9cad128d124f86bafe48d2cd11b5eee74bec77d9\n")
        f.write(
            "# format: case_type rel_gap px py vx vy radius gx gy v_pref theta num_humans "
            "(px py vx vy radius)*num_humans chosen_vx chosen_vy\n")
        for label, group in (("adversarial", adversarial), ("typical", typical)):
            for s in group:
                ss = s["self_state"]
                fields = [
                    label, f64(s["rel_gap"]),
                    f64(ss.px), f64(ss.py), f64(ss.vx), f64(ss.vy), f64(ss.radius),
                    f64(ss.gx), f64(ss.gy), f64(ss.v_pref),
                    # theta is set to 0.0 explicitly, not JointState's mutated
                    # atan2(vy,vx) artifact: rotate()'s holonomic branch always outputs
                    # theta=0 regardless of input (verified against cadrl.py - S4.7), so the
                    # mutation is functionally inert but confusing to a fixture reader.
                    f64(0.0),
                    str(len(s["humans"])),
                ]
                for h in s["humans"]:
                    fields += [f64(h.px), f64(h.py), f64(h.vx), f64(h.vy), f64(h.radius)]
                fields += [f64(s["action"].vx), f64(s["action"].vy)]
                f.write(" ".join(fields) + "\n")

    print(f"Wrote {len(adversarial)} adversarial + {len(typical)} typical cases to {out_path}")
    print(f"adversarial rel_gaps: {[round(float(s['rel_gap']), 6) for s in adversarial]}")
    print(f"typical rel_gaps: {[round(float(s['rel_gap']), 6) for s in typical]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
