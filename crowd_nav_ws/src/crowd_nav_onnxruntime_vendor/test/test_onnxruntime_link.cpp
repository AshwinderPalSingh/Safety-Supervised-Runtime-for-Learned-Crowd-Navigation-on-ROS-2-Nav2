// Phase 0 plumbing check ( Phase 0 / Phase 6): confirms the vendored
// ONNX Runtime actually links and runs inference, not just that CMake found headers.
// Model is a single Linear(4,1) with weight=1, bias=0, so output == sum(input) — a
// known-answer check, not just "did it crash."
#include <onnxruntime_cxx_api.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <path-to-trivial.onnx>\n", argv[0]);
    return 1;
  }

  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "crowd_nav_onnxruntime_vendor_test");
  Ort::SessionOptions session_options;
  Ort::Session session(env, argv[1], session_options);

  std::array<float, 4> input_values{1.0f, 2.0f, 3.0f, 4.0f};
  std::array<int64_t, 2> input_shape{1, 4};

  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info, input_values.data(), input_values.size(), input_shape.data(), input_shape.size());

  const char * input_names[] = {"input"};
  const char * output_names[] = {"output"};

  auto output_tensors = session.Run(
    Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

  float result = output_tensors.front().GetTensorMutableData<float>()[0];
  float expected = 1.0f + 2.0f + 3.0f + 4.0f;

  std::printf("ONNX Runtime inference result: %f (expected %f)\n", result, expected);

  if (std::fabs(result - expected) > 1e-3f) {
    std::fprintf(stderr, "FAIL: result does not match expected sum\n");
    return 1;
  }

  std::printf("PASS\n");
  return 0;
}
