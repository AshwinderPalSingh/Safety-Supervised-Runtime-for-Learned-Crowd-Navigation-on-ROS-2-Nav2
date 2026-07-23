#ifndef CROWD_NAV_POLICY_ADAPTERS__ONNX_INFERENCE_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__ONNX_INFERENCE_HPP_

#include "onnxruntime_cxx_api.h"

#include "crowd_nav_policy_adapters/policy_adapter.hpp"

namespace crowd_nav_policy_adapters
{

// Runs `session` on `inputs` (whose names must match the session's actual input names, in
// order - see shape_validation.hpp) and returns the outputs as a TensorBundle keyed by
// `output_names`. Thin wrapper around Ort::Session::Run() so PolicyAdapter implementations
// (DummyAdapter now, SarlAdapter in Phase 8) don't each hand-roll ONNX Runtime's C++ API.
TensorBundle runInference(
  Ort::Session & session,
  const TensorBundle & inputs,
  const std::vector<std::string> & output_names);

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__ONNX_INFERENCE_HPP_
