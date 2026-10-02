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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "prediction_ml/mtr_inference_engine.hpp"

namespace
{

class TemporaryMetadataFile
{
public:
  explicit TemporaryMetadataFile(const std::string & contents)
  {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() / ("prediction_ml_mtr_contract_" + std::to_string(suffix) + ".yaml");
    std::ofstream output(path_);
    output << contents;
    output.close();
    if (!output) {
      throw std::runtime_error("failed to create temporary MTR metadata file");
    }
  }

  TemporaryMetadataFile(const TemporaryMetadataFile &) = delete;
  TemporaryMetadataFile & operator=(const TemporaryMetadataFile &) = delete;

  ~TemporaryMetadataFile()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  std::string path() const
  {
    return path_.string();
  }

private:
  std::filesystem::path path_;
};

constexpr const char * kEightTargetContract = R"yaml(---
inputs:
  - name: obj_trajs
    dtype: float32
    shape: [8, 128, 11, 29]
  - name: obj_trajs_mask
    dtype: bool
    shape: [8, 128, 11]
  - name: obj_trajs_last_pos
    dtype: float32
    shape: [8, 128, 3]
  - name: map_polylines
    dtype: float32
    shape: [8, 768, 20, 9]
  - name: map_polylines_mask
    dtype: bool
    shape: [8, 768, 20]
  - name: map_polylines_center
    dtype: float32
    shape: [8, 768, 3]
  - name: track_index_to_predict
    dtype: int64
    shape: [8]
  - name: center_type_ids
    dtype: int64
    shape: [8]
outputs:
  - name: pred_scores
    dtype: float32
    shape: [8, 6]
  - name: pred_trajs
    dtype: float32
    shape: [8, 6, 80, 7]
)yaml";

prediction_ml::MtrModelContract makeSmallContract()
{
  prediction_ml::MtrModelContract contract;
  contract.inputs = {{"obj_trajs", "float32", {8, 128, 11, 29}}, {"obj_trajs_mask", "bool", {8, 128, 11}}};
  contract.outputs = {{"pred_scores", "float32", {8, 6}}, {"pred_trajs", "float32", {8, 6, 80, 7}}};
  return contract;
}

}  // namespace

TEST(MtrInferenceEngineFactory, DisabledModeReturnsNullEngine)
{
  prediction_ml::MtrConfig cfg;
  cfg.mode = prediction_ml::MtrMode::Disabled;

  auto engine = prediction_ml::createMtrInferenceEngine(cfg);

  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  EXPECT_FALSE(engine->lastError().empty());
}

TEST(MtrInferenceEngineFactory, NullModeReturnsNullEngine)
{
  prediction_ml::MtrConfig cfg;
  cfg.mode = prediction_ml::MtrMode::Null;

  auto engine = prediction_ml::createMtrInferenceEngine(cfg);

  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  prediction_ml::MtrInputTensors in;
  auto out = engine->infer(in);
  EXPECT_FALSE(out.valid);
}

TEST(MtrInferenceEngineFactory, TensorRtModeReturnsEngine)
{
  prediction_ml::MtrConfig cfg;
  cfg.mode = prediction_ml::MtrMode::TensorRt;

  auto engine = prediction_ml::createMtrInferenceEngine(cfg);

  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  EXPECT_FALSE(engine->lastError().empty());
}

#ifdef PREDICTION_ML_ENABLE_TENSORRT
TEST(TensorRtBackendInitialization, RejectsEmptyEnginePath)
{
  prediction_ml::MtrConfig cfg;
  cfg.mode = prediction_ml::MtrMode::TensorRt;

  auto engine = prediction_ml::createMtrInferenceEngine(cfg);

  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  EXPECT_NE(engine->lastError().find("mtr.engine_path is empty"), std::string::npos);
}

TEST(TensorRtBackendInitialization, RejectsEmptyMetadataPath)
{
  prediction_ml::MtrConfig cfg;
  cfg.mode = prediction_ml::MtrMode::TensorRt;
  cfg.engine_path = "/unused/nonempty/mtr.engine";

  auto engine = prediction_ml::createMtrInferenceEngine(cfg);

  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  EXPECT_NE(engine->lastError().find("MTR metadata path is empty"), std::string::npos);
}

TEST(TensorRtBackendInitialization, RejectsMissingEngineAndPreservesError)
{
  const TemporaryMetadataFile metadata(kEightTargetContract);
  prediction_ml::MtrConfig cfg;
  cfg.mode = prediction_ml::MtrMode::TensorRt;
  cfg.engine_path = "/path/that/does/not/exist/mtr.engine";
  cfg.metadata_path = metadata.path();

  auto engine = prediction_ml::createMtrInferenceEngine(cfg);

  ASSERT_NE(engine, nullptr);
  EXPECT_FALSE(engine->ready());
  const std::string initialization_error = engine->lastError();
  EXPECT_NE(initialization_error.find("failed to open TensorRT engine"), std::string::npos);

  prediction_ml::MtrInputTensors input;
  input.valid = true;
  const auto output = engine->infer(input);
  EXPECT_FALSE(output.valid);
  EXPECT_EQ(engine->lastError(), initialization_error);
}
#endif

TEST(MtrModelContractLoader, LoadsEightTargetMetadata)
{
  const TemporaryMetadataFile metadata(kEightTargetContract);

  const auto contract = prediction_ml::loadMtrModelContract(metadata.path());

  ASSERT_EQ(contract.inputs.size(), 8u);
  EXPECT_EQ(contract.inputs[0].name, "obj_trajs");
  EXPECT_EQ(contract.inputs[0].dtype, "float32");
  EXPECT_EQ(contract.inputs[0].shape, (std::vector<int64_t>{8, 128, 11, 29}));
  EXPECT_EQ(contract.inputs[6].name, "track_index_to_predict");
  EXPECT_EQ(contract.inputs[6].dtype, "int64");
  ASSERT_EQ(contract.outputs.size(), 2u);
  EXPECT_EQ(contract.outputs[1].name, "pred_trajs");
  EXPECT_EQ(contract.outputs[1].shape, (std::vector<int64_t>{8, 6, 80, 7}));
}

TEST(MtrModelContractLoader, NormalizesSupportedDtypeAliases)
{
  const TemporaryMetadataFile metadata(R"yaml(
inputs:
  - name: input
    dtype: FP32
    shape: [8, 1]
outputs:
  - name: output
    dtype: boolean
    shape: [8]
)yaml");

  const auto contract = prediction_ml::loadMtrModelContract(metadata.path());

  EXPECT_EQ(contract.inputs[0].dtype, "float32");
  EXPECT_EQ(contract.outputs[0].dtype, "bool");
}

TEST(MtrModelContractLoader, RejectsEmptyOrMissingPath)
{
  EXPECT_THROW(prediction_ml::loadMtrModelContract(""), std::runtime_error);
  EXPECT_THROW(prediction_ml::loadMtrModelContract("/path/that/does/not/exist/mtr.yaml"), std::runtime_error);
}

TEST(MtrModelContractLoader, RejectsIncompleteTensorEntry)
{
  const TemporaryMetadataFile metadata(R"yaml(
inputs:
  - name: obj_trajs
    dtype: float32
outputs:
  - name: pred_scores
    dtype: float32
    shape: [8, 6]
)yaml");

  EXPECT_THROW(prediction_ml::loadMtrModelContract(metadata.path()), std::runtime_error);
}

TEST(MtrModelContractLoader, RejectsMalformedShape)
{
  const TemporaryMetadataFile metadata(R"yaml(
inputs:
  - name: obj_trajs
    dtype: float32
    shape: [8, 0, 11, 29]
outputs:
  - name: pred_scores
    dtype: float32
    shape: [8, 6]
)yaml");

  EXPECT_THROW(prediction_ml::loadMtrModelContract(metadata.path()), std::runtime_error);
}

TEST(MtrModelContractLoader, RejectsUnknownField)
{
  const TemporaryMetadataFile metadata(R"yaml(
inputs:
  - name: obj_trajs
    dtype: float32
    shape: [8, 128, 11, 29]
    layout: row_major
outputs:
  - name: pred_scores
    dtype: float32
    shape: [8, 6]
)yaml");

  EXPECT_THROW(prediction_ml::loadMtrModelContract(metadata.path()), std::runtime_error);
}

TEST(MtrModelContractValidator, AcceptsMatchingBindingsInDifferentOrder)
{
  const auto expected = makeSmallContract();
  auto actual = makeSmallContract();
  std::swap(actual.inputs[0], actual.inputs[1]);
  std::swap(actual.outputs[0], actual.outputs[1]);

  std::string error;
  EXPECT_TRUE(prediction_ml::validateMtrModelContract(expected, actual, error));
  EXPECT_TRUE(error.empty());
}

TEST(MtrModelContractValidator, AcceptsDynamicDimensionWildcard)
{
  auto expected = makeSmallContract();
  auto actual = makeSmallContract();
  expected.inputs[0].shape[0] = -1;

  std::string error;
  EXPECT_TRUE(prediction_ml::validateMtrModelContract(expected, actual, error));
  EXPECT_TRUE(error.empty());
}

TEST(MtrModelContractValidator, RejectsEmptyExpectedContract)
{
  prediction_ml::MtrModelContract expected;
  prediction_ml::MtrModelContract actual;

  std::string error;
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, actual, error));
  EXPECT_FALSE(error.empty());
}

TEST(MtrModelContractValidator, RejectsMissingOrExtraBinding)
{
  const auto expected = makeSmallContract();
  auto missing = makeSmallContract();
  missing.inputs.pop_back();

  std::string error;
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, missing, error));
  EXPECT_NE(error.find("count mismatch"), std::string::npos);

  auto extra = makeSmallContract();
  extra.outputs.push_back({"debug_output", "float32", {1}});
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, extra, error));
  EXPECT_NE(error.find("count mismatch"), std::string::npos);
}

TEST(MtrModelContractValidator, RejectsDtypeMismatch)
{
  const auto expected = makeSmallContract();
  auto actual = makeSmallContract();
  actual.inputs[1].dtype = "uint8";

  std::string error;
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, actual, error));
  EXPECT_NE(error.find("dtype mismatch"), std::string::npos);
}

TEST(MtrModelContractValidator, RejectsRankAndShapeMismatch)
{
  const auto expected = makeSmallContract();
  auto wrong_rank = makeSmallContract();
  wrong_rank.outputs[1].shape = {8, 6, 80};

  std::string error;
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, wrong_rank, error));
  EXPECT_NE(error.find("rank mismatch"), std::string::npos);

  auto wrong_shape = makeSmallContract();
  wrong_shape.outputs[1].shape = {8, 6, 79, 7};
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, wrong_shape, error));
  EXPECT_NE(error.find("shape mismatch"), std::string::npos);
}

TEST(MtrModelContractValidator, RejectsDuplicateBindingName)
{
  const auto expected = makeSmallContract();
  auto actual = makeSmallContract();
  actual.inputs[1].name = actual.inputs[0].name;

  std::string error;
  EXPECT_FALSE(prediction_ml::validateMtrModelContract(expected, actual, error));
  EXPECT_NE(error.find("duplicate actual"), std::string::npos);
}
