#include <mid360_ray_preprocessor/tf_stamp_cache.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

namespace mrp = mid360_ray_preprocessor;

TEST(TfStampCache, ComputesMaximumGapAcrossCoveringBrackets) {
  mrp::TfStampCache cache(16U);
  for (const std::uint64_t stamp : {0U, 20U, 40U, 90U, 110U}) {
    cache.observe(stamp);
  }

  const auto result = cache.observeWindow(30U, 100U);
  EXPECT_TRUE(result.bracketed);
  EXPECT_EQ(result.status, mrp::TfStampBracketStatus::BRACKETED);
  EXPECT_EQ(result.left_bracket_ns, 20U);
  EXPECT_EQ(result.right_bracket_ns, 110U);
  EXPECT_EQ(result.maximum_adjacent_gap_ns, 50U);
  EXPECT_EQ(result.covering_sample_count, 4U);
  EXPECT_STREQ(mrp::tfStampBracketStatusName(result.status), "bracketed");
}

TEST(TfStampCache, DistinguishesMissingSidesAndInvalidWindow) {
  mrp::TfStampCache cache(8U);
  cache.observe(20U);
  cache.observe(40U);

  EXPECT_EQ(cache.observeWindow(10U, 30U).status,
            mrp::TfStampBracketStatus::MISSING_LEFT);
  EXPECT_EQ(cache.observeWindow(30U, 50U).status,
            mrp::TfStampBracketStatus::MISSING_RIGHT);
  EXPECT_EQ(cache.observeWindow(50U, 40U).status,
            mrp::TfStampBracketStatus::INVALID_WINDOW);
  EXPECT_EQ(cache.observeWindow(10U, 50U).status,
            mrp::TfStampBracketStatus::MISSING_BOTH);

  mrp::TfStampCache empty(2U);
  EXPECT_EQ(empty.observeWindow(0U, 0U).status,
            mrp::TfStampBracketStatus::NO_SAMPLES);
}

TEST(TfStampCache, ExactSampleBracketsAZeroWidthWindow) {
  mrp::TfStampCache cache(4U);
  cache.observe(100U);
  const auto result = cache.observeWindow(100U, 100U);
  ASSERT_TRUE(result.bracketed);
  EXPECT_EQ(result.left_bracket_ns, 100U);
  EXPECT_EQ(result.right_bracket_ns, 100U);
  EXPECT_EQ(result.maximum_adjacent_gap_ns, 0U);
  EXPECT_EQ(result.covering_sample_count, 1U);
}

TEST(TfStampCache, CountsDuplicateAndOutOfOrderArrivalsIndependently) {
  mrp::TfStampCache cache(8U);
  cache.observe(20U);
  cache.observe(40U);
  cache.observe(20U);  // Old duplicate: both duplicate and out of order.
  cache.observe(30U);  // New unique sample delivered out of order.

  const auto result = cache.observeWindow(20U, 40U);
  ASSERT_TRUE(result.bracketed);
  EXPECT_EQ(result.duplicate_stamp_count, 1U);
  EXPECT_EQ(result.out_of_order_stamp_count, 2U);
  EXPECT_EQ(result.cached_unique_sample_count, 3U);
  EXPECT_EQ(result.maximum_adjacent_gap_ns, 10U);
}

TEST(TfStampCache, RetainsOnlyNewestBoundedSourceStamps) {
  mrp::TfStampCache cache(3U);
  cache.observe(10U);
  cache.observe(20U);
  cache.observe(30U);
  cache.observe(40U);

  const auto retained = cache.observeWindow(20U, 40U);
  ASSERT_TRUE(retained.bracketed);
  EXPECT_EQ(retained.cached_unique_sample_count, 3U);
  EXPECT_EQ(retained.evicted_stamp_count, 1U);
  EXPECT_EQ(retained.left_bracket_ns, 20U);
  EXPECT_EQ(retained.right_bracket_ns, 40U);
  EXPECT_EQ(cache.observeWindow(10U, 20U).status,
            mrp::TfStampBracketStatus::MISSING_LEFT);
}

TEST(TfStampCache, ClearStartsANewObservationGeneration) {
  mrp::TfStampCache cache(4U);
  cache.observe(30U);
  cache.observe(30U);
  cache.observe(20U);
  cache.clear();

  const auto result = cache.observeWindow(0U, 0U);
  EXPECT_EQ(result.status, mrp::TfStampBracketStatus::NO_SAMPLES);
  EXPECT_EQ(result.cached_unique_sample_count, 0U);
  EXPECT_EQ(result.duplicate_stamp_count, 0U);
  EXPECT_EQ(result.out_of_order_stamp_count, 0U);
  EXPECT_EQ(result.evicted_stamp_count, 0U);
}

TEST(TfStampCache, RejectsCapacitySmallerThanTwo) {
  EXPECT_THROW(mrp::TfStampCache(1U), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
