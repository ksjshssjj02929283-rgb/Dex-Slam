#include "cartographer/localization/internal/2d/pose_initializer_2d.h"

#include <limits>
#include <memory>
#include <vector>

#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/localization/internal/2d/pose_initializer_options_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace localization {

// Exercise the production validator and synchronous entry points without a
// background thread. Control the existing BBS consistency gate explicitly;
// fixing its stale reference in local mode is outside this regression's scope.
class PoseInitializerTest : public ::testing::Test {
 protected:
  proto::PoseInitializerOptions2D Options() {
    auto resolver = std::make_unique<common::ConfigurationFileResolver>(
        std::vector<std::string>{std::string(common::kSourceDirectory) +
                                 "/configuration_files"});
    common::LuaParameterDictionary dict(
        "include 'pose_initializer_2d.lua'\nreturn POSE_INITIALIZER_2D",
        std::move(resolver));
    auto options = CreatePoseInitializerOptions2D(&dict);
    options.set_global_matcher_type(proto::GLOBAL_MATCHER_ORIGINAL);
    options.set_use_online_correlative_scan_matching(false);
    auto* fast = options.mutable_fast_correlative_scan_matcher_options();
    fast->set_linear_search_window(0.1);
    fast->set_angular_search_window(0.01);
    fast->set_branch_and_bound_depth(2);
    options.mutable_ceres_scan_matcher_options()->set_translation_weight(100.);
    options.mutable_ceres_scan_matcher_options()->set_rotation_weight(100.);
    return options;
  }

  std::shared_ptr<mapping::Submap2D> Map(bool all_known = false) {
    // Nonzero, asymmetric bounds: x in (6, 14), y in (18, 24).
    auto grid = std::make_unique<mapping::ProbabilityGrid>(
        mapping::MapLimits(0.1, Eigen::Vector2d(14., 24.),
                           mapping::CellLimits(60, 80)),
        &tables_);
    if (all_known) {
      for (int x = 0; x < 60; ++x) {
        for (int y = 0; y < 80; ++y) {
          grid->SetProbability(Eigen::Array2i(x, y), 0.9f);
        }
      }
    }
    return std::make_shared<mapping::Submap2D>(
        Eigen::Vector2f(14., 24.), std::move(grid), &tables_,
        std::make_unique<sensor::PointCloud>(),
        std::make_unique<sensor::PointCloud>(), 0,
        std::map<int, mapping::MapLine2D>{});
  }

  void Mark(const std::shared_ptr<mapping::Submap2D>& map,
            const Eigen::Vector2d& position, float probability) {
    auto* grid = const_cast<mapping::ProbabilityGrid*>(
        static_cast<const mapping::ProbabilityGrid*>(map->grid()));
    grid->SetProbability(grid->limits().GetCellIndex(position.cast<float>()),
                         probability);
  }

  void Prepare(PoseInitializer* initializer, const Eigen::Vector2d& position) {
    const transform::Rigid2d pose(position, 0.);
    initializer->global_pose_estimate_ =
        std::make_shared<transform::Rigid2d>(pose);
    initializer->last_global_bbs_pose_ = pose;
    initializer->UpdateSensorData(sensor::PointCloud({{{1.05f, 0.05f, 0.f}},
                                                      {{-1.05f, -0.05f, 0.f}},
                                                      {{0.05f, 1.05f, 0.f}}}),
                                  {});
  }

  bool Validate(PoseInitializer* initializer) {
    return initializer->IsRelocalizationSuccess();
  }

  float Score(PoseInitializer* initializer) {
    return initializer->ScorePoseAgainstGrid(
        *initializer->global_pose_estimate_, *initializer->curr_cloud_,
        *initializer->global_submap_->grid());
  }

  void RunLocal(PoseInitializer* initializer) {
    initializer->RelocalzationOnce();
  }

  bool HasEstimate(const PoseInitializer& initializer) {
    return initializer.global_pose_estimate_ != nullptr;
  }

  mapping::ValueConversionTables tables_;
};

TEST_F(PoseInitializerTest, AcceptsKnownFreeAndOccupiedCells) {
  for (const float probability : {0.1f, 0.9f}) {
    PoseInitializer initializer(Options());
    auto matching = Map(true);
    auto static_map = Map();
    const Eigen::Vector2d position(11.25, 21.25);
    Mark(static_map, position, probability);
    initializer.InitGlobalSubmap(matching, static_map);
    Prepare(&initializer, position);
    EXPECT_TRUE(Validate(&initializer)) << probability;
  }
}

TEST_F(PoseInitializerTest, RejectsUnknownPositionDespiteHighEndpointScore) {
  PoseInitializer initializer(Options());
  auto map = Map();
  const Eigen::Vector2d position(11.25, 21.25);
  Prepare(&initializer, position);
  Mark(map, position + Eigen::Vector2d(1.05, 0.05), 0.9f);
  Mark(map, position + Eigen::Vector2d(-1.05, -0.05), 0.9f);
  Mark(map, position + Eigen::Vector2d(0.05, 1.05), 0.9f);
  initializer.InitGlobalSubmap(map, map);
  ASSERT_GT(Score(&initializer), 0.8f);
  EXPECT_FALSE(Validate(&initializer));
}

TEST_F(PoseInitializerTest, UsesStaticMapInsteadOfKnownDynamicMatchingMap) {
  PoseInitializer initializer(Options());
  initializer.InitGlobalSubmap(Map(true), Map());
  Prepare(&initializer, Eigen::Vector2d(11.25, 21.25));
  ASSERT_GT(Score(&initializer), 0.8f);
  EXPECT_FALSE(Validate(&initializer));
}

TEST_F(PoseInitializerTest, ChecksAllFourBoundariesInMapCoordinates) {
  PoseInitializer initializer(Options());
  // Enough endpoints remain inside the map to pass the existing score gate.
  auto map = Map(true);
  initializer.InitGlobalSubmap(map, map);
  for (const Eigen::Vector2d& position :
       {Eigen::Vector2d(6.01, 21.25), Eigen::Vector2d(13.99, 21.25),
        Eigen::Vector2d(11.25, 18.01), Eigen::Vector2d(11.25, 23.99)}) {
    Prepare(&initializer, position);
    EXPECT_TRUE(Validate(&initializer)) << position.transpose();
  }
  for (const Eigen::Vector2d& position :
       {Eigen::Vector2d(5.99, 21.25), Eigen::Vector2d(14.01, 21.25),
        Eigen::Vector2d(11.25, 17.99), Eigen::Vector2d(11.25, 24.01)}) {
    Prepare(&initializer, position);
    EXPECT_FALSE(Validate(&initializer)) << position.transpose();
  }
}

TEST_F(PoseInitializerTest, RejectsMissingStaticMapAndGrid) {
  PoseInitializer initializer(Options());
  auto matching = Map(true);
  initializer.InitGlobalSubmap(matching, nullptr);
  Prepare(&initializer, Eigen::Vector2d(11.25, 21.25));
  EXPECT_FALSE(Validate(&initializer));
  auto no_grid = std::make_shared<mapping::Submap2D>(
      Eigen::Vector2f(14., 24.), nullptr, &tables_,
      std::make_unique<sensor::PointCloud>(),
      std::make_unique<sensor::PointCloud>(), 0,
      std::map<int, mapping::MapLine2D>{});
  initializer.InitGlobalSubmap(matching, no_grid);
  EXPECT_FALSE(Validate(&initializer));
}

TEST_F(PoseInitializerTest, RejectsNonFiniteAndExtremePositions) {
  PoseInitializer initializer(Options());
  auto map = Map(true);
  initializer.InitGlobalSubmap(map, map);
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::max(),
                             -std::numeric_limits<double>::max()}) {
    Prepare(&initializer, Eigen::Vector2d(value, 21.25));
    EXPECT_FALSE(Validate(&initializer));
  }
}

TEST_F(PoseInitializerTest, ReplacesStaticMapAfterStopAndMapSwitch) {
  PoseInitializer initializer(Options());
  auto known = Map(true);
  initializer.InitGlobalSubmap(known, known);
  Prepare(&initializer, Eigen::Vector2d(11.25, 21.25));
  ASSERT_TRUE(Validate(&initializer));
  initializer.stop();
  initializer.InitGlobalSubmap(known, Map());
  Prepare(&initializer, Eigen::Vector2d(11.25, 21.25));
  EXPECT_FALSE(Validate(&initializer));
  initializer.InitGlobalSubmap(known, known);
  EXPECT_TRUE(Validate(&initializer));
}

TEST_F(PoseInitializerTest, LocalEntryRejectsUnknownAndClearsEstimate) {
  PoseInitializer initializer(Options());
  auto matching = Map();
  const Eigen::Vector2d position(11.25, 21.25);
  Mark(matching, position + Eigen::Vector2d(1.05, 0.05), 0.9f);
  Mark(matching, position + Eigen::Vector2d(-1.05, -0.05), 0.9f);
  Mark(matching, position + Eigen::Vector2d(0.05, 1.05), 0.9f);
  initializer.Initialize();
  initializer.InitGlobalSubmap(matching, Map(true));
  Prepare(&initializer, Eigen::Vector2d(11.25, 21.25));
  initializer.SetInitialPose(transform::Rigid2d({11.25, 21.25}, 0.));
  RunLocal(&initializer);
  ASSERT_EQ(initializer.GetInitializerStatus(), PoseInitializerStatus::Success);
  initializer.InitGlobalSubmap(matching, Map());
  initializer.SetInitialPose(transform::Rigid2d({11.25, 21.25}, 0.));
  RunLocal(&initializer);
  EXPECT_EQ(initializer.GetInitializerStatus(), PoseInitializerStatus::Failed);
  EXPECT_FALSE(HasEstimate(initializer));
}

TEST_F(PoseInitializerTest, GlobalEntryRejectsUnknownAndClearsEstimate) {
  PoseInitializer initializer(Options());
  auto matching = Map(true);
  initializer.Initialize();
  initializer.InitGlobalSubmap(matching, matching);
  Prepare(&initializer, Eigen::Vector2d(11.25, 21.25));
  initializer.TriggerGlobalRelocalization();
  initializer.GlobalRelocalzationOnce();
  ASSERT_EQ(initializer.GetInitializerStatus(), PoseInitializerStatus::Success);
  initializer.InitGlobalSubmap(matching, Map());
  initializer.TriggerGlobalRelocalization();
  initializer.GlobalRelocalzationOnce();
  EXPECT_EQ(initializer.GetInitializerStatus(), PoseInitializerStatus::Failed);
  EXPECT_FALSE(HasEstimate(initializer));
}

}  // namespace localization
}  // namespace cartographer
