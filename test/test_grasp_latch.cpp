#include <gtest/gtest.h>

#include "stonefish_net/core/grasp_latch.hpp"

namespace
{
using stonefish_net::core::GraspLatch;
using stonefish_net::core::GraspPositionIntent;
using Action = GraspLatch::Action;
using Direction = GraspPositionIntent::Direction;

TEST(GraspPositionIntent, OpeningAfterFurtherClosureUsesTheMostClosedTarget)
{
  GraspPositionIntent intent;
  EXPECT_EQ(intent.update(0.0, -0.2, 1, 0.03), Direction::Closing);
  EXPECT_EQ(intent.update(0.125, -0.1, 1, 0.03), Direction::Closing);
  // Still more closed than the original capture angle, but this is opening.
  EXPECT_EQ(intent.update(0.05, 0.12, 1, 0.03), Direction::Opening);
}

TEST(GraspPositionIntent, TheOppositeFingerUsesItsOwnClosingSign)
{
  GraspPositionIntent intent;
  EXPECT_EQ(intent.update(0.0, 0.2, -1, 0.03), Direction::Closing);
  EXPECT_EQ(intent.update(-0.125, 0.1, -1, 0.03), Direction::Closing);
  EXPECT_EQ(intent.update(-0.05, -0.12, -1, 0.03), Direction::Opening);
}

TEST(GraspPositionIntent, SmallOpeningCommandsAccumulateAcrossPositionUpdates)
{
  GraspPositionIntent intent;
  EXPECT_EQ(intent.update(0, 0, 1, 0.03), Direction::None);
  EXPECT_EQ(intent.update(-0.01, -0.01, 1, 0.03), Direction::None);
  EXPECT_EQ(intent.update(-0.02, -0.02, 1, 0.03), Direction::None);
  EXPECT_EQ(intent.update(-0.031, -0.031, 1, 0.03), Direction::Opening);
}

TEST(GraspPositionIntent, ARepeatedTargetDoesNotFollowServoTrackingError)
{
  GraspPositionIntent intent;
  ASSERT_EQ(intent.update(0.1, 0, 1, 0.03), Direction::Closing);
  EXPECT_EQ(intent.update(0.1, -0.1, 1, 0.03), Direction::None);
  EXPECT_EQ(intent.update(0.1, 0.2, 1, 0.03), Direction::None);
}

TEST(GraspPositionIntent, RepeatingAnOpeningTargetCannotRearmTheLatch)
{
  GraspPositionIntent intent;
  GraspLatch latch;
  ASSERT_EQ(intent.update(0.1, 0, 1, 0.03), Direction::Closing);
  ASSERT_EQ(latch.update(true, false, true, true), Action::Attach);
  ASSERT_EQ(intent.update(0.0, 0.1, 1, 0.03), Direction::Opening);
  ASSERT_EQ(latch.update(false, true, true, true), Action::Release);
  // The finger overshoots the opening target. Its tracking error must not be
  // mistaken for a fresh closing command toward the repeated position target.
  EXPECT_EQ(intent.update(0.0, -0.1, 1, 0.03), Direction::None);
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspPositionIntent, ANewClosingTargetAfterOpeningCanRearm)
{
  GraspPositionIntent intent;
  ASSERT_EQ(intent.update(0.1, 0, 1, 0.03), Direction::Closing);
  ASSERT_EQ(intent.update(0, 0.1, 1, 0.03), Direction::Opening);
  EXPECT_EQ(intent.update(0.02, 0.01, 1, 0.03), Direction::Closing);
}

TEST(GraspPositionIntent, ResetComparesTheNextPositionCommandAgainstTheActualAngle)
{
  GraspPositionIntent intent;
  ASSERT_EQ(intent.update(-0.2, -0.3, 1, 0.03), Direction::Closing);
  // Velocity control subsequently closed the finger farther than that target.
  intent.reset();
  EXPECT_EQ(intent.update(0, 0.1, 1, 0.03), Direction::Opening);
  EXPECT_EQ(intent.update(0, -0.1, 1, 0.03), Direction::None);
}

TEST(GraspLatch, AnInitiallyClosedGripperDoesNotAttachWithoutAClosingCommand)
{
  GraspLatch latch;
  EXPECT_FALSE(latch.attached());
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, ClosingAroundAnObjectAttachesExactlyOnce)
{
  GraspLatch latch;
  EXPECT_EQ(latch.update(true, false, true, true), Action::Attach);
  EXPECT_TRUE(latch.attached());
  EXPECT_EQ(latch.update(true, false, true, true), Action::None);
  EXPECT_TRUE(latch.attached());
}

TEST(GraspLatch, ClosingCannotAttachAnObjectOutsideTheFingers)
{
  GraspLatch latch;
  EXPECT_EQ(latch.update(true, false, false, true), Action::None);
  EXPECT_EQ(latch.update(false, false, false, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, AGripperThatIsStillOpenCannotAttach)
{
  GraspLatch latch;
  EXPECT_EQ(latch.update(true, false, true, false), Action::None);
  EXPECT_EQ(latch.update(false, false, true, false), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, AZeroCommandPreservesTheArmedClosingAttempt)
{
  GraspLatch latch;
  EXPECT_EQ(latch.update(true, false, false, false), Action::None);
  EXPECT_EQ(latch.update(false, false, true, false), Action::None);
  EXPECT_EQ(latch.update(false, false, true, true), Action::Attach);
  EXPECT_TRUE(latch.attached());
}

TEST(GraspLatch, AttachmentSurvivesLossOfProximityAndClosureMeasurements)
{
  GraspLatch latch;
  ASSERT_EQ(latch.update(true, false, true, true), Action::Attach);
  EXPECT_EQ(latch.update(false, false, false, false), Action::None);
  EXPECT_EQ(latch.update(true, false, false, false), Action::None);
  EXPECT_TRUE(latch.attached());
}

TEST(GraspLatch, OpeningReleasesExactlyOnceAndPreventsImmediateReattachment)
{
  GraspLatch latch;
  ASSERT_EQ(latch.update(true, false, true, true), Action::Attach);
  EXPECT_EQ(latch.update(false, true, true, true), Action::Release);
  EXPECT_FALSE(latch.attached());
  EXPECT_EQ(latch.update(false, true, true, true), Action::None);
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, OpeningHasPriorityOverClosingBeforeAttachment)
{
  GraspLatch latch;
  EXPECT_EQ(latch.update(true, true, true, true), Action::None);
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, OpeningHasPriorityOverClosingWhileAttached)
{
  GraspLatch latch;
  ASSERT_EQ(latch.update(true, false, true, true), Action::Attach);
  EXPECT_EQ(latch.update(true, true, true, true), Action::Release);
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, OpeningCancelsAnArmedAttemptBeforeItCanAttach)
{
  GraspLatch latch;
  ASSERT_EQ(latch.update(true, false, false, false), Action::None);
  EXPECT_EQ(latch.update(false, true, false, false), Action::None);
  EXPECT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_FALSE(latch.attached());
}

TEST(GraspLatch, ANewClosingCommandCanReattachAfterRelease)
{
  GraspLatch latch;
  ASSERT_EQ(latch.update(true, false, true, true), Action::Attach);
  ASSERT_EQ(latch.update(false, true, true, true), Action::Release);
  ASSERT_EQ(latch.update(false, false, true, true), Action::None);
  EXPECT_EQ(latch.update(true, false, true, true), Action::Attach);
  EXPECT_TRUE(latch.attached());
}
}  // namespace
