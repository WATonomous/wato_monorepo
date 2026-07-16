// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "prediction_ml/mtr_backend.hpp"

namespace prediction_ml
{

namespace
{

std::string shapeToString(const std::vector<int64_t> & shape)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t i = 0; i < shape.size(); ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << shape[i];
  }
  stream << "]";
  return stream.str();
}

bool dimensionMatches(const int64_t expected, const int64_t actual)
{
  // Keep the validator usable for future dynamic engines. The current 8-target
  // metadata uses only fixed, positive dimensions.
  return expected == actual || expected == -1;
}

bool validateTensorSpecs(
  const std::vector<MtrTensorSpec> & expected,
  const std::vector<MtrTensorSpec> & actual,
  const std::string & binding_kind,
  std::string & error)
{
  if (expected.empty()) {
    error = "expected MTR " + binding_kind + " contract is empty";
    return false;
  }
  if (actual.size() != expected.size()) {
    error = "MTR " + binding_kind + " count mismatch: expected " +
      std::to_string(expected.size()) + ", got " + std::to_string(actual.size());
    return false;
  }

  std::map<std::string, const MtrTensorSpec *> actual_by_name;
  for (const auto & spec : actual) {
    if (spec.name.empty()) {
      error = "actual MTR " + binding_kind + " contains an empty binding name";
      return false;
    }
    if (!actual_by_name.emplace(spec.name, &spec).second) {
      error = "duplicate actual MTR " + binding_kind + " binding name '" + spec.name + "'";
      return false;
    }
  }

  std::map<std::string, bool> expected_names;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    const auto & expected_spec = expected[i];
    if (expected_spec.name.empty()) {
      error = "expected MTR " + binding_kind + " contains an empty binding name";
      return false;
    }
    if (!expected_names.emplace(expected_spec.name, true).second) {
      error = "duplicate expected MTR " + binding_kind + " binding name '" +
        expected_spec.name + "'";
      return false;
    }

    const auto actual_it = actual_by_name.find(expected_spec.name);
    if (actual_it == actual_by_name.end()) {
      error = binding_kind + "[" + std::to_string(i) + "] missing actual binding '" +
        expected_spec.name + "'";
      return false;
    }
    const auto & actual_spec = *actual_it->second;

    if (expected_spec.dtype != actual_spec.dtype) {
      error = binding_kind + " '" + expected_spec.name + "' dtype mismatch: expected '" +
        expected_spec.dtype + "', got '" + actual_spec.dtype + "'";
      return false;
    }
    if (expected_spec.shape.size() != actual_spec.shape.size()) {
      error = binding_kind + " '" + expected_spec.name + "' rank mismatch: expected " +
        shapeToString(expected_spec.shape) + ", got " + shapeToString(actual_spec.shape);
      return false;
    }
    for (std::size_t dimension = 0; dimension < expected_spec.shape.size(); ++dimension) {
      if (!dimensionMatches(expected_spec.shape[dimension], actual_spec.shape[dimension])) {
        error = binding_kind + " '" + expected_spec.name + "' shape mismatch at dimension " +
          std::to_string(dimension) + ": expected " + shapeToString(expected_spec.shape) +
          ", got " + shapeToString(actual_spec.shape);
        return false;
      }
    }
  }

  return true;
}

}  // namespace

#ifdef PREDICTION_ML_ENABLE_TENSORRT
namespace detail
{
std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config);
}  // namespace detail
#endif

NullMtrInferenceEngine::NullMtrInferenceEngine(MtrConfig config)
: config_(std::move(config))
{}

bool NullMtrInferenceEngine::ready() const
{
  return false;
}

std::string NullMtrInferenceEngine::lastError() const
{
  return last_error_;
}

MtrOutputTensors NullMtrInferenceEngine::infer(const MtrInputTensors & /*input*/)
{
  MtrOutputTensors out;
  out.valid = false;
  return out;
}

std::unique_ptr<IMtrInferenceEngine> createMtrInferenceEngine(const MtrConfig & config)
{
#ifdef PREDICTION_ML_ENABLE_TENSORRT
  if (config.mode == MtrMode::TensorRt) {
    return detail::createTensorRtMtrInferenceEngine(config);
  }
#endif

  // Disabled, null, and unavailable TensorRT modes all use fallback-only output.
  return std::make_unique<NullMtrInferenceEngine>(config);
}

MtrMode parseMtrMode(const std::string & mode_str)
{
  std::string s = mode_str;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  if (s == "null") {
    return MtrMode::Null;
  }
  if (s == "tensorrt" || s == "trt") {
    return MtrMode::TensorRt;
  }
  return MtrMode::Disabled;
}

MtrModelContract loadMtrModelContract(const std::string & metadata_path)
{
  // This intentionally supports only the small YAML subset used by the MTR
  // contract files. Keeping it local avoids adding yaml-cpp to CPU-only builds.
  if (metadata_path.empty()) {
    throw std::runtime_error("MTR metadata path is empty");
  }

  std::ifstream file(metadata_path);
  if (!file.is_open()) {
    throw std::runtime_error("failed to open MTR metadata file: " + metadata_path);
  }

  const auto trim = [](const std::string & value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
      return std::string{};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
  };

  const auto unquote = [&trim](const std::string & value) {
    std::string result = trim(value);
    if (result.size() >= 2 &&
      ((result.front() == '"' && result.back() == '"') ||
      (result.front() == '\'' && result.back() == '\'')))
    {
      result = result.substr(1, result.size() - 2);
    }
    return result;
  };

  const auto normalizeDtype = [&unquote](const std::string & value) {
    std::string dtype = unquote(value);
    std::transform(
      dtype.begin(), dtype.end(), dtype.begin(),
      [](unsigned char character) { return std::tolower(character); });
    if (dtype == "fp32" || dtype == "float") {
      return std::string{"float32"};
    }
    if (dtype == "int64_t") {
      return std::string{"int64"};
    }
    if (dtype == "boolean") {
      return std::string{"bool"};
    }
    return dtype;
  };

  const auto parseShape = [&trim](const std::string & value, const std::size_t line_number) {
    const std::string text = trim(value);
    if (text.size() < 2 || text.front() != '[' || text.back() != ']') {
      throw std::runtime_error(
        "invalid MTR metadata shape at line " + std::to_string(line_number) +
        ": expected [d0, d1, ...]");
    }

    std::vector<int64_t> shape;
    std::stringstream dimensions(text.substr(1, text.size() - 2));
    std::string token;
    while (std::getline(dimensions, token, ',')) {
      token = trim(token);
      if (token.empty()) {
        throw std::runtime_error(
          "empty MTR metadata dimension at line " + std::to_string(line_number));
      }
      std::size_t consumed = 0;
      int64_t dimension = 0;
      try {
        dimension = std::stoll(token, &consumed);
      } catch (const std::exception &) {
        throw std::runtime_error(
          "invalid MTR metadata dimension '" + token + "' at line " +
          std::to_string(line_number));
      }
      if (consumed != token.size() || dimension == 0 || dimension < -1) {
        throw std::runtime_error(
          "invalid MTR metadata dimension '" + token + "' at line " +
          std::to_string(line_number));
      }
      shape.push_back(dimension);
    }
    if (shape.empty()) {
      throw std::runtime_error(
        "empty MTR metadata shape at line " + std::to_string(line_number));
    }
    return shape;
  };

  enum class Section { None, Inputs, Outputs };
  Section section = Section::None;
  bool saw_inputs = false;
  bool saw_outputs = false;
  MtrModelContract contract;
  MtrTensorSpec current;
  bool has_current = false;
  bool has_name = false;
  bool has_dtype = false;
  bool has_shape = false;

  const auto appendCurrent = [&]() {
    if (!has_current) {
      return;
    }
    if (!has_name || !has_dtype || !has_shape) {
      throw std::runtime_error(
        "incomplete MTR tensor entry in metadata file: " + metadata_path);
    }
    if (section == Section::Inputs) {
      if (std::any_of(
          contract.inputs.begin(), contract.inputs.end(),
          [&current](const MtrTensorSpec & spec) { return spec.name == current.name; }))
      {
        throw std::runtime_error("duplicate MTR input binding name '" + current.name + "'");
      }
      contract.inputs.push_back(current);
    } else if (section == Section::Outputs) {
      if (std::any_of(
          contract.outputs.begin(), contract.outputs.end(),
          [&current](const MtrTensorSpec & spec) { return spec.name == current.name; }))
      {
        throw std::runtime_error("duplicate MTR output binding name '" + current.name + "'");
      }
      contract.outputs.push_back(current);
    } else {
      throw std::runtime_error(
        "MTR tensor entry appears outside inputs/outputs in: " + metadata_path);
    }
    current = MtrTensorSpec{};
    has_current = false;
    has_name = false;
    has_dtype = false;
    has_shape = false;
  };

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    line = trim(line);
    if (line.empty() || line == "---") {
      continue;
    }

    if (line == "inputs:" || line == "outputs:") {
      appendCurrent();
      if (line == "inputs:") {
        if (saw_inputs) {
          throw std::runtime_error(
            "duplicate inputs section at line " + std::to_string(line_number));
        }
        saw_inputs = true;
        section = Section::Inputs;
      } else {
        if (saw_outputs) {
          throw std::runtime_error(
            "duplicate outputs section at line " + std::to_string(line_number));
        }
        saw_outputs = true;
        section = Section::Outputs;
      }
      continue;
    }
    if (section == Section::None) {
      throw std::runtime_error(
        "unexpected MTR metadata content at line " + std::to_string(line_number) + ": " + line);
    }

    if (line.rfind("- ", 0) == 0) {
      appendCurrent();
      has_current = true;
      line = trim(line.substr(2));
    } else if (!has_current) {
      throw std::runtime_error(
        "expected '-' tensor entry at line " + std::to_string(line_number) + ": " + line);
    }

    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error(
        "invalid MTR metadata field at line " + std::to_string(line_number));
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    if (value.empty()) {
      throw std::runtime_error(
        "empty MTR metadata field '" + key + "' at line " + std::to_string(line_number));
    }

    if (key == "name") {
      if (has_name) {
        throw std::runtime_error("duplicate tensor name at line " + std::to_string(line_number));
      }
      current.name = unquote(value);
      has_name = !current.name.empty();
    } else if (key == "dtype") {
      if (has_dtype) {
        throw std::runtime_error("duplicate tensor dtype at line " + std::to_string(line_number));
      }
      current.dtype = normalizeDtype(value);
      has_dtype = !current.dtype.empty();
    } else if (key == "shape") {
      if (has_shape) {
        throw std::runtime_error("duplicate tensor shape at line " + std::to_string(line_number));
      }
      current.shape = parseShape(value, line_number);
      has_shape = true;
    } else {
      throw std::runtime_error(
        "unknown MTR metadata field '" + key + "' at line " +
        std::to_string(line_number));
    }
  }
  appendCurrent();

  if (!saw_inputs || !saw_outputs || contract.inputs.empty() || contract.outputs.empty()) {
    throw std::runtime_error(
      "MTR metadata must define non-empty inputs and outputs: " + metadata_path);
  }
  return contract;
}

bool validateMtrModelContract(
  const MtrModelContract & expected, const MtrModelContract & actual, std::string & error)
{
  error.clear();
  if (!validateTensorSpecs(expected.inputs, actual.inputs, "input", error)) {
    return false;
  }
  if (!validateTensorSpecs(expected.outputs, actual.outputs, "output", error)) {
    return false;
  }
  return true;
}

}  // namespace prediction_ml
