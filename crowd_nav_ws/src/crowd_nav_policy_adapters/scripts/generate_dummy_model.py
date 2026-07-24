#!/usr/bin/env python3
"""Generates Phase 6's trivial ONNX model (IMPLEMENTATION_PLAN.md S4.3.1/S3 Phase 6).

Reads config/policy_adapter.yaml - the SAME file DummyAdapter reads at construction time - so
the model's declared input/output shape is computed from config, never hand-typed as a literal
on either side. That's the whole point: shape validation should catch a real drift between the
model file and the runtime config, not just check the dummy against itself.

Internals are a single Linear(feature_dim, 1) with weight=1, bias=0, so output == sum(input)
per row - a known-correct answer to check inference against, not just "did it crash" (same
pattern as crowd_nav_onnxruntime_vendor's Phase 0 trivial.onnx smoke test).

The exported shape is STATIC (no dynamic_axes) - deliberately, so shape validation has a
concrete batch dimension to check on both sides, not a dynamic one that trivially "matches
anything."

Usage: python3 generate_dummy_model.py [path/to/policy_adapter.yaml] [output.onnx]
Defaults to the paths inside this package's own config/ and models/ directories.
"""
import sys
from pathlib import Path

import torch
import torch.nn as nn
import yaml


class TrivialValueNet(nn.Module):
    def __init__(self, feature_dim):
        super().__init__()
        self.linear = nn.Linear(feature_dim, 1)
        with torch.no_grad():
            self.linear.weight.fill_(1.0)
            self.linear.bias.fill_(0.0)

    def forward(self, x):
        return self.linear(x)


def main():
    here = Path(__file__).resolve().parent.parent
    config_path = (
        Path(sys.argv[1]) if len(sys.argv) > 1 else here / "config" / "policy_adapter.yaml")
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else here / "models" / "dummy_policy.onnx"

    with open(config_path) as f:
        config = yaml.safe_load(f)

    num_candidates = config["speed_samples"] * config["rotation_samples"] + 1
    feature_dim = 9 + 5 * config["max_humans"]

    model = TrivialValueNet(feature_dim)
    model.eval()

    example_input = torch.zeros((num_candidates, feature_dim), dtype=torch.float32)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model,
        (example_input,),
        str(out_path),
        input_names=["candidates"],
        output_names=["value"],
        # No dynamic_axes: shape is baked in as (num_candidates, feature_dim) from
        # example_input, matching config exactly - see module docstring.
    )
    print(
        f"Wrote {out_path} - input 'candidates' [{num_candidates}, {feature_dim}], "
        f"output 'value' [{num_candidates}, 1]"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
