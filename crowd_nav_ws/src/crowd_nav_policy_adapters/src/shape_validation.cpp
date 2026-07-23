#include "crowd_nav_policy_adapters/shape_validation.hpp"

#include <functional>
#include <sstream>
#include <string>
#include <vector>

namespace crowd_nav_policy_adapters
{

namespace
{

std::string shapeToString(const std::vector<int64_t> & shape)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < shape.size(); ++i) {
    oss << shape[i];
    if (i + 1 < shape.size()) {
      oss << ", ";
    }
  }
  oss << "]";
  return oss.str();
}

void checkOneSide(
  const std::vector<std::string> & expected_names,
  const std::vector<std::vector<int64_t>> & expected_shapes,
  size_t actual_count,
  const std::string & side_label,
  const std::function<std::string(size_t)> & actual_name_at,
  const std::function<std::vector<int64_t>(size_t)> & actual_shape_at)
{
  if (expected_names.size() != actual_count) {
    throw ShapeValidationError(
      "expected " + std::to_string(expected_names.size()) + " " + side_label +
      "(s) but the session has " + std::to_string(actual_count));
  }

  for (size_t i = 0; i < expected_names.size(); ++i) {
    const std::string actual_name = actual_name_at(i);
    if (actual_name != expected_names[i]) {
      throw ShapeValidationError(
        side_label + " " + std::to_string(i) + ": expected name '" + expected_names[i] +
        "' but the session has '" + actual_name + "'");
    }

    const std::vector<int64_t> actual_shape = actual_shape_at(i);
    const std::vector<int64_t> & expected_shape = expected_shapes[i];
    if (actual_shape.size() != expected_shape.size()) {
      throw ShapeValidationError(
        side_label + " '" + expected_names[i] + "': expected rank " +
        std::to_string(expected_shape.size()) + " (" + shapeToString(expected_shape) +
        ") but the session has rank " + std::to_string(actual_shape.size()) + " (" +
        shapeToString(actual_shape) + ")");
    }
    for (size_t d = 0; d < expected_shape.size(); ++d) {
      const bool dynamic_ok = expected_shape[d] == -1;
      if (!dynamic_ok && expected_shape[d] != actual_shape[d]) {
        throw ShapeValidationError(
          side_label + " '" + expected_names[i] + "' dim " + std::to_string(d) +
          ": expected " + std::to_string(expected_shape[d]) + " but the session has " +
          std::to_string(actual_shape[d]) + " (full expected " + shapeToString(expected_shape) +
          ", full actual " + shapeToString(actual_shape) + ")");
      }
    }
  }
}

}  // namespace

void validateSessionShapes(Ort::Session & session, const ShapeSpec & expected)
{
  Ort::AllocatorWithDefaultOptions allocator;

  checkOneSide(
    expected.input_names, expected.input_shapes, session.GetInputCount(), "input",
    [&](size_t i) {
      auto name = session.GetInputNameAllocated(i, allocator);
      return std::string(name.get());
    },
    [&](size_t i) {
      return session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    });

  checkOneSide(
    expected.output_names, expected.output_shapes, session.GetOutputCount(), "output",
    [&](size_t i) {
      auto name = session.GetOutputNameAllocated(i, allocator);
      return std::string(name.get());
    },
    [&](size_t i) {
      return session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    });
}

}  // namespace crowd_nav_policy_adapters
