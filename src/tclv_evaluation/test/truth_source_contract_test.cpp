#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include <tclv_evaluation/truth_source_contract.hpp>

namespace te = tclv_evaluation;

namespace {

std::vector<te::TruthSourceContractEntry> sources(
    std::initializer_list<const char*> ids) {
  std::vector<te::TruthSourceContractEntry> result;
  for (const char* id : ids) {
    result.push_back({id});
  }
  return result;
}

std::vector<te::TruthTargetContractEntry> targets(
    std::initializer_list<std::pair<const char*, const char*>> entries) {
  std::vector<te::TruthTargetContractEntry> result;
  for (const auto& entry : entries) {
    result.push_back({entry.first, entry.second});
  }
  return result;
}

TEST(TruthSourceContract, AcceptsOneOrMoreCompleteTargetSources) {
  EXPECT_NO_THROW(te::validateTruthSourceContract(
      sources({"uav1", "uav2"}), targets({{"uav2", "uav2"}})));
  EXPECT_NO_THROW(te::validateTruthSourceContract(
      sources({"uav1", "uav2", "uav3"}),
      targets({{"uav2", "uav2"}, {"uav3", "uav3"}})));
  EXPECT_NO_THROW(te::validateTruthSourceContract(
      sources({"uav3", "uav1", "uav4", "uav2"}),
      targets({{"rear", "uav3"}, {"left", "uav4"}, {"front", "uav2"}})));
}

TEST(TruthSourceContract, RejectsMissingOrTargetOnlyObserver) {
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav2", "uav3"}),
                   targets({{"uav2", "uav2"}, {"uav3", "uav3"}})),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1"}), targets({{"uav2", "uav2"}})),
               std::invalid_argument);
}

TEST(TruthSourceContract, RejectsDuplicateTruthSourceIds) {
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2", "uav2"}),
                   targets({{"uav2", "uav2"}})),
               std::invalid_argument);
}

TEST(TruthSourceContract, RejectsOrphanTruthSource) {
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2", "uav3"}),
                   targets({{"uav2", "uav2"}})),
               std::invalid_argument);
}

TEST(TruthSourceContract, RejectsUnknownOrDuplicateTargets) {
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}),
                   targets({{"uav9", "uav9"}})),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2", "uav3"}),
                   targets({{"same", "uav2"}, {"same", "uav3"}})),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}),
                   targets({{"first", "uav2"}, {"second", "uav2"}})),
               std::invalid_argument);
}

TEST(TruthSourceContract, RejectsObserverAsTargetIdOrTruthSource) {
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}),
                   targets({{"uav1", "uav2"}})),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}),
                   targets({{"observer_alias", "uav1"}})),
               std::invalid_argument);
}

TEST(TruthSourceContract, RejectsEmptyTargetSetAndEmptyIds) {
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}), {}),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", ""}), targets({{"uav2", "uav2"}})),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}), targets({{"", "uav2"}})),
               std::invalid_argument);
  EXPECT_THROW(te::validateTruthSourceContract(
                   sources({"uav1", "uav2"}), targets({{"uav2", ""}})),
               std::invalid_argument);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
