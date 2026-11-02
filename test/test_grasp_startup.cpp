#include <gtest/gtest.h>

#include "stonefish_net/core/grasp_startup.hpp"

namespace
{
using stonefish_net::core::GraspStartup;
using State = GraspStartup::State;

TEST(GraspStartup, AnOpenGripperCannotCompleteInitialization)
{
  GraspStartup startup;
  for (int i = 0; i < 1999; ++i) EXPECT_EQ(startup.update(0.001, {0, 0}, true), State::Closing);
  EXPECT_EQ(startup.update(0.001, {0, 0}, true), State::Failed);
}

TEST(GraspStartup, BothFingersMustExertPressure)
{
  GraspStartup startup;
  for (int i = 0; i < 50; ++i) {
    EXPECT_EQ(startup.update(0.001, {1.0, 0}, true), State::Closing);
    EXPECT_EQ(startup.update(0.001, {0, 1.0}, true), State::Closing);
  }
  for (int i = 0; i < 19; ++i) EXPECT_EQ(startup.update(0.001, {0.2, 0.3}, true), State::Closing);
  EXPECT_EQ(startup.update(0.001, {0.2, 0.3}, true), State::Ready);
}

TEST(GraspStartup, AnImpactDoesNotCountAsAStableGrasp)
{
  GraspStartup startup;
  for (int i = 0; i < 50; ++i) EXPECT_EQ(startup.update(0.001, {2.0, 2.0}, false), State::Closing);
  for (int i = 0; i < 19; ++i) EXPECT_EQ(startup.update(0.001, {0.1, 0.1}, true), State::Closing);
  EXPECT_EQ(startup.update(0.001, {0.1, 0.1}, true), State::Ready);
}

TEST(GraspStartup, LossOfContactResetsTheLoadedInterval)
{
  GraspStartup startup;
  for (int i = 0; i < 15; ++i) startup.update(0.001, {0.2, 0.2}, true);
  EXPECT_EQ(startup.update(0.001, {0.2, 0.001}, true), State::Closing);
  for (int i = 0; i < 19; ++i) EXPECT_EQ(startup.update(0.001, {0.2, 0.2}, true), State::Closing);
  EXPECT_EQ(startup.update(0.001, {0.2, 0.2}, true), State::Ready);
}

TEST(GraspStartup, InitializationDoesNotReengageAfterTheUserOpensTheGripper)
{
  GraspStartup startup;
  for (int i = 0; i < 20; ++i) startup.update(0.001, {0.2, 0.2}, true);
  EXPECT_EQ(startup.state(), State::Ready);
  EXPECT_EQ(startup.update(3.0, {0, 0}, false), State::Ready);
}

TEST(GraspStartup, FailedInitializationDoesNotLaterReportSuccess)
{
  GraspStartup startup;
  EXPECT_EQ(startup.update(2.0, {0, 0}, true), State::Failed);
  EXPECT_EQ(startup.update(0.020, {1, 1}, true), State::Failed);
}
}  // namespace
