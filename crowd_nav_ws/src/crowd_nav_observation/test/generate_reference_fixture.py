#!/usr/bin/env python3
"""Generates the round-trip reference fixture for Phase 5's observation-builder test.
Calls the ACTUAL reference repo's CADRL.rotate() (used
unchanged by SARL) on synthetic (self_state + one_human_state) rows, not a transcription of the
formula - this is what makes the C++ test a check against the real implementation rather than
against this project's own understanding of it.

Run once, offline, against a checkout of tkkim-robot/Gazebo-CrowdNav (commit
9cad128d124f86bafe48d2cd11b5eee74bec77d9, matching the checkpoint's source repo - see
). The output is checked into the repo
(fixtures/sarl_rotate_reference.txt) so CI/the C++ test never needs network access or a
reference-repo checkout at test time - only this generation step does, and it's not run as
part of the normal build.

Format is deliberately plain whitespace-delimited text, not JSON: one case per line, self_state
(9 values) then human_state (5 values) then rotated (13 values), 27 numbers per line - avoids
pulling in a JSON parsing dependency for the C++ test just to read five lines of floats.

Usage: python3 generate_reference_fixture.py /path/to/cloned/Gazebo-CrowdNav
"""
import sys


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} /path/to/cloned/Gazebo-CrowdNav", file=sys.stderr)
        return 1
    repo_path = sys.argv[1]
    sys.path.insert(0, repo_path)

    import torch  # noqa: E402
    from crowd_nav.policy.cadrl import CADRL  # noqa: E402

    policy = CADRL()
    policy.kinematics = 'holonomic'  # matches the checkpoint

    # Each case: (self_state 9-tuple, human_state 5-tuple), covering a spread of headings,
    # distances, and velocities so the round-trip test isn't just exercising one code path.
    # self_state order: px,py,vx,vy,radius,gx,gy,v_pref,theta
    # human_state order: px,py,vx,vy,radius
    cases = [
        ((0.0, 0.0, 1.0, 0.0, 0.3, 5.0, 0.0, 1.0, 0.0), (2.0, 1.0, -0.3, 0.1, 0.3)),
        ((1.0, 2.0, 0.0, 1.0, 0.3, 1.0, 6.0, 1.0, 0.0), (1.5, 3.0, 0.2, -0.2, 0.3)),
        ((-2.0, 0.5, -0.7, 0.7, 0.25, -6.0, 0.5, 1.0, 0.0), (-4.0, 1.0, 0.5, 0.0, 0.35)),
        ((0.0, 0.0, 0.0, 0.0, 0.3, 3.0, 4.0, 1.0, 0.0), (0.1, 0.1, 0.0, 0.0, 0.3)),
        ((5.0, -5.0, 0.6, -0.8, 0.3, -5.0, 5.0, 1.0, 0.0), (7.0, -3.0, -0.4, 0.4, 0.4)),
    ]

    out_path = "fixtures/sarl_rotate_reference.txt"
    with open(out_path, "w") as f:
        f.write(
            "# source_repo=tkkim-robot/Gazebo-CrowdNav "
            "source_commit=9cad128d124f86bafe48d2cd11b5eee74bec77d9\n"
        )
        f.write(
            "# columns: px py vx vy radius gx gy v_pref theta "
            "(self, 9) | px1 py1 vx1 vy1 radius1 (human, 5) | "
            "dg v_pref theta radius vx vy px1 py1 vx1 vy1 radius1 da radius_sum (rotated, 13)\n"
        )
        n = 0
        for self_state, human_state in cases:
            row = self_state + human_state
            state_tensor = torch.Tensor([row])
            rotated = policy.rotate(state_tensor).squeeze(0).tolist()
            values = list(self_state) + list(human_state) + rotated
            f.write(" ".join(repr(v) for v in values) + "\n")
            n += 1
    print(f"Wrote {n} cases to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
