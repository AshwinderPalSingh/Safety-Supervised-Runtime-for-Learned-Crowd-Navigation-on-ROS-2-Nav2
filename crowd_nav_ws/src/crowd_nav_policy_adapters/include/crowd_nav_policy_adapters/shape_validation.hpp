#ifndef CROWD_NAV_POLICY_ADAPTERS__SHAPE_VALIDATION_HPP_
#define CROWD_NAV_POLICY_ADAPTERS__SHAPE_VALIDATION_HPP_

#include <stdexcept>

#include "onnxruntime_cxx_api.h"  // NOLINT(build/include_subdir)

#include "crowd_nav_policy_adapters/policy_adapter.hpp"

namespace crowd_nav_policy_adapters
{

class ShapeValidationError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// IMPLEMENTATION_PLAN.md S4.3: "At load time, the controller plugin queries the ONNX Runtime
// session's actual input/output tensor names and shapes and asserts they match
// expectedShape() - mismatch fails the lifecycle transition loudly rather than segfaulting or
// silently misinterpreting a tensor." Checks input/output count, names (order-sensitive - ONNX
// Runtime's Run() call matches tensors to names positionally against the arrays passed in, so
// a silently-reordered name would misfeed a tensor without either side raising an error), and
// per-dimension shape (-1 in `expected` matches any extent at that position, but the dimension
// COUNT must still match - a rank mismatch is a real mismatch, not "dynamic"). Throws
// ShapeValidationError with the exact mismatch on failure.
void validateSessionShapes(Ort::Session & session, const ShapeSpec & expected);

}  // namespace crowd_nav_policy_adapters

#endif  // CROWD_NAV_POLICY_ADAPTERS__SHAPE_VALIDATION_HPP_
