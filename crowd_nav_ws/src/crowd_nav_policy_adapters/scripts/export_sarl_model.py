#!/usr/bin/env python3
"""Exports the real SARL value network (IMPLEMENTATION_PLAN.md S3 Phase 8 / S4.7) from the
pinned checkpoint to ONNX, with a static-shape-free (dynamic batch + num_humans axes) graph -
SarlAdapter builds an unpadded per-decision batch, not a fixed-max-humans one (S4.7's
no-padding finding: the network's masked-softmax attention only excludes a human row when its
raw attention score is exactly 0.0, a property no padding convention can guarantee).

Drops the checkpoint's own debug line (`self.attention_weights = ...cpu().numpy()`, in
crowd_nav/policy/sarl.py's ValueNetwork.forward()) via a wrapper that reuses the checkpoint's
own loaded submodules (mlp1/mlp2/mlp3/attention) unchanged - verified bit-identical against the
original checkpoint model on real varied joint states before ever exporting (this script
re-checks that on every run, not just once by hand).

Run once, offline, against a checkout of tkkim-robot/Gazebo-CrowdNav (commit
9cad128d124f86bafe48d2cd11b5eee74bec77d9, matching il_model.pth's source - S1.8). Output
(models/sarl_value_net.onnx) is checked into the repo so the C++ test/runtime never needs
PyTorch or a reference-repo checkout - only this generation step does.

Usage: python3 export_sarl_model.py /path/to/cloned/Gazebo-CrowdNav
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
    import torch.nn as nn  # noqa: E402
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
    policy.time_step = 0.25  # env.config [env] time_step, verified (S1.8/S4.7)

    original_model = policy.get_model()

    class ExportableValueNetwork(nn.Module):
        """Identical to sarl.py's ValueNetwork.forward() minus the non-exportable debug line
        (an instance-attribute assignment that never feeds the returned value)."""

        def __init__(self, m):
            super().__init__()
            self.self_state_dim = m.self_state_dim
            self.global_state_dim = m.global_state_dim
            self.mlp1 = m.mlp1
            self.mlp2 = m.mlp2
            self.with_global_state = m.with_global_state
            self.attention = m.attention
            self.mlp3 = m.mlp3

        def forward(self, state):
            size = state.shape
            self_state = state[:, 0, :self.self_state_dim]
            mlp1_output = self.mlp1(state.view((-1, size[2])))
            mlp2_output = self.mlp2(mlp1_output)
            if self.with_global_state:
                global_state = torch.mean(mlp1_output.view(size[0], size[1], -1), 1, keepdim=True)
                global_state = global_state.expand(
                    (size[0], size[1], self.global_state_dim)
                ).contiguous().view(-1, self.global_state_dim)
                attention_input = torch.cat([mlp1_output, global_state], dim=1)
            else:
                attention_input = mlp1_output
            scores = self.attention(attention_input).view(size[0], size[1], 1).squeeze(dim=2)
            scores_exp = torch.exp(scores) * (scores != 0).float()
            weights = (scores_exp / torch.sum(scores_exp, dim=1, keepdim=True)).unsqueeze(2)
            features = mlp2_output.view(size[0], size[1], -1)
            weighted_feature = torch.sum(torch.mul(weights, features), dim=1)
            joint_state = torch.cat([self_state, weighted_feature], dim=1)
            return self.mlp3(joint_state)

    wrapper = ExportableValueNetwork(original_model)
    wrapper.eval()

    def make_joint_state(n_humans, seed):
        rng = np.random.RandomState(seed)
        self_state = FullState(
            rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-1, 1), rng.uniform(-1, 1),
            0.3, rng.uniform(-3, 3), rng.uniform(-3, 3), 1.0, 0.0)
        humans = [
            ObservableState(
                rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-1, 1), rng.uniform(-1, 1),
                0.3)
            for _ in range(n_humans)
        ]
        return JointState(self_state, humans)

    def to_rotated(js):
        row = torch.cat(
            [torch.Tensor([(js.self_state.px, js.self_state.py, js.self_state.vx, js.self_state.vy,
                            js.self_state.radius, js.self_state.gx, js.self_state.gy,
                            js.self_state.v_pref, js.self_state.theta,
                            h.px, h.py, h.vx, h.vy, h.radius)])
             for h in js.human_states], dim=0)
        return policy.rotate(row)

    max_diff = 0.0
    example_rotated = None
    for n_humans in (1, 3, 5):
        js = make_joint_state(n_humans, seed=n_humans)
        rotated = to_rotated(js).unsqueeze(0)
        if example_rotated is None:
            example_rotated = rotated
        with torch.no_grad():
            out_original = original_model(rotated)
            out_wrapper = wrapper(rotated)
        max_diff = max(max_diff, (out_original - out_wrapper).abs().max().item())

    print(f"max |original - wrapper| before export = {max_diff:.3e}")
    if max_diff > 1e-6:
        print("ABORTING: wrapper does not match the original checkpoint model", file=sys.stderr)
        return 1

    models_dir = Path(__file__).resolve().parent.parent / "models"
    models_dir.mkdir(parents=True, exist_ok=True)
    out_path = models_dir / "sarl_value_net.onnx"

    torch.onnx.export(
        wrapper, (example_rotated,), str(out_path),
        input_names=["rotated_batch"], output_names=["value"],
        dynamic_axes={
            "rotated_batch": {0: "batch", 1: "num_humans"},
            "value": {0: "batch"},
        },
        opset_version=18,
    )
    print(f"Wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
