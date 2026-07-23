// Deliberately includes cases built to FAIL under a validator that just rubber-stamps
// agreement - a validator that never rejects anything would pass a "does it accept the correct
// shape" test trivially. See IMPLEMENTATION_PLAN.md S3 Phase 6's "deliberately-mismatched
// expectedShape()" done-bar item.
#include <string>

#include "gtest/gtest.h"

#include "crowd_nav_policy_adapters/shape_validation.hpp"

using crowd_nav_policy_adapters::ShapeSpec;
using crowd_nav_policy_adapters::ShapeValidationError;
using crowd_nav_policy_adapters::validateSessionShapes;

namespace
{
std::string modelPath()
{
  return std::string(CROWD_NAV_POLICY_ADAPTERS_PACKAGE_DIR) + "/models/dummy_policy.onnx";
}

ShapeSpec correctShape()
{
  ShapeSpec spec;
  spec.input_names = {"candidates"};
  spec.input_shapes = {{81, 34}};
  spec.output_names = {"value"};
  spec.output_shapes = {{81, 1}};
  return spec;
}

Ort::Env & sharedEnv()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_shape_validation");
  return env;
}

Ort::Session makeSession()
{
  Ort::SessionOptions options;
  return Ort::Session(sharedEnv(), modelPath().c_str(), options);
}
}  // namespace

TEST(ValidateSessionShapes, AcceptsTheActualCorrectShape)
{
  Ort::Session session = makeSession();
  EXPECT_NO_THROW(validateSessionShapes(session, correctShape()));
}

TEST(ValidateSessionShapes, AcceptsDynamicBatchDimensionMarker)
{
  Ort::Session session = makeSession();
  ShapeSpec spec = correctShape();
  spec.input_shapes[0][0] = -1;  // "any batch size" should still match this model's fixed 81
  EXPECT_NO_THROW(validateSessionShapes(session, spec));
}

TEST(ValidateSessionShapes, RejectsWrongFeatureDim)
{
  Ort::Session session = makeSession();
  ShapeSpec spec = correctShape();
  spec.input_shapes[0][1] = 13;  // real SARL's rotated width, NOT this model's raw 34
  EXPECT_THROW(validateSessionShapes(session, spec), ShapeValidationError);
}

TEST(ValidateSessionShapes, RejectsWrongCandidateCount)
{
  Ort::Session session = makeSession();
  ShapeSpec spec = correctShape();
  spec.input_shapes[0][0] = 80;  // off-by-one on the candidate count
  EXPECT_THROW(validateSessionShapes(session, spec), ShapeValidationError);
}

TEST(ValidateSessionShapes, RejectsWrongInputName)
{
  Ort::Session session = makeSession();
  ShapeSpec spec = correctShape();
  spec.input_names[0] = "wrong_name";
  EXPECT_THROW(validateSessionShapes(session, spec), ShapeValidationError);
}

TEST(ValidateSessionShapes, RejectsWrongOutputRank)
{
  Ort::Session session = makeSession();
  ShapeSpec spec = correctShape();
  spec.output_shapes[0] = {81};  // missing the trailing size-1 dim
  EXPECT_THROW(validateSessionShapes(session, spec), ShapeValidationError);
}

TEST(ValidateSessionShapes, RejectsMissingExtraInput)
{
  Ort::Session session = makeSession();
  ShapeSpec spec = correctShape();
  spec.input_names.push_back("second_input");
  spec.input_shapes.push_back({81, 1});
  EXPECT_THROW(validateSessionShapes(session, spec), ShapeValidationError);
}
