#include "crowd_nav_policy_adapters/onnx_inference.hpp"

#include <functional>
#include <numeric>
#include <vector>

namespace crowd_nav_policy_adapters
{

TensorBundle runInference(
  Ort::Session & session,
  const TensorBundle & inputs,
  const std::vector<std::string> & output_names)
{
  Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  std::vector<Ort::Value> input_tensors;
  std::vector<const char *> input_name_ptrs;
  input_tensors.reserve(inputs.names.size());
  input_name_ptrs.reserve(inputs.names.size());
  for (size_t i = 0; i < inputs.names.size(); ++i) {
    input_tensors.push_back(
      Ort::Value::CreateTensor<float>(
        memory_info,
        const_cast<float *>(inputs.data[i].data()), inputs.data[i].size(),
        inputs.shapes[i].data(), inputs.shapes[i].size()));
    input_name_ptrs.push_back(inputs.names[i].c_str());
  }

  std::vector<const char *> output_name_ptrs;
  output_name_ptrs.reserve(output_names.size());
  for (const auto & n : output_names) {
    output_name_ptrs.push_back(n.c_str());
  }

  auto output_tensors = session.Run(
    Ort::RunOptions{nullptr}, input_name_ptrs.data(), input_tensors.data(), input_tensors.size(),
    output_name_ptrs.data(), output_name_ptrs.size());

  TensorBundle result;
  for (size_t i = 0; i < output_tensors.size(); ++i) {
    auto shape = output_tensors[i].GetTensorTypeAndShapeInfo().GetShape();
    const size_t count = static_cast<size_t>(
      std::accumulate(shape.begin(), shape.end(), int64_t{1}, std::multiplies<int64_t>()));
    const float * raw = output_tensors[i].GetTensorData<float>();

    result.names.push_back(output_names[i]);
    result.shapes.push_back(shape);
    result.data.emplace_back(raw, raw + count);
  }
  return result;
}

}  // namespace crowd_nav_policy_adapters
