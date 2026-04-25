package org.teamtitanium.subsystems.shooter.turret;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class TurretWrapLogicTest {
  private static final double MIN_RAD = Units.degreesToRadians(-360.0);
  private static final double MAX_RAD = Units.degreesToRadians(60.0);

  private static final double ENTRY_MARGIN_RAD = Units.degreesToRadians(12.0);
  private static final double EXIT_MARGIN_RAD = Units.degreesToRadians(18.0);
  private static final double MIN_COMMAND_DELTA_RAD = Units.degreesToRadians(25.0);
  private static final double NEAR_TARGET_DELTA_RAD = Units.degreesToRadians(7.0);

  @Test
  void detectsWrapFromMaxToMinBoundary() {
    Turret.WrapEvaluation evaluation =
        Turret.evaluateWrappingState(
            Units.degreesToRadians(58.0),
            Units.degreesToRadians(-355.0),
            false,
            MIN_RAD,
            MAX_RAD,
            ENTRY_MARGIN_RAD,
            EXIT_MARGIN_RAD,
            MIN_COMMAND_DELTA_RAD,
            NEAR_TARGET_DELTA_RAD);

    assertTrue(evaluation.wrappingThisCycle());
    assertTrue(evaluation.wrappingAfterUpdate());
  }

  @Test
  void detectsWrapFromMinToMaxBoundary() {
    Turret.WrapEvaluation evaluation =
        Turret.evaluateWrappingState(
            Units.degreesToRadians(-358.0),
            Units.degreesToRadians(58.0),
            false,
            MIN_RAD,
            MAX_RAD,
            ENTRY_MARGIN_RAD,
            EXIT_MARGIN_RAD,
            MIN_COMMAND_DELTA_RAD,
            NEAR_TARGET_DELTA_RAD);

    assertTrue(evaluation.wrappingThisCycle());
    assertTrue(evaluation.wrappingAfterUpdate());
  }

  @Test
  void doesNotWrapForLargeMoveAwayFromBoundaries() {
    Turret.WrapEvaluation evaluation =
        Turret.evaluateWrappingState(
            Units.degreesToRadians(-120.0),
            Units.degreesToRadians(-220.0),
            false,
            MIN_RAD,
            MAX_RAD,
            ENTRY_MARGIN_RAD,
            EXIT_MARGIN_RAD,
            MIN_COMMAND_DELTA_RAD,
            NEAR_TARGET_DELTA_RAD);

    assertFalse(evaluation.wrappingThisCycle());
    assertFalse(evaluation.wrappingAfterUpdate());
  }

  @Test
  void doesNotWrapForSmallBoundaryMotion() {
    Turret.WrapEvaluation evaluation =
        Turret.evaluateWrappingState(
            Units.degreesToRadians(59.0),
            Units.degreesToRadians(52.0),
            false,
            MIN_RAD,
            MAX_RAD,
            ENTRY_MARGIN_RAD,
            EXIT_MARGIN_RAD,
            MIN_COMMAND_DELTA_RAD,
            NEAR_TARGET_DELTA_RAD);

    assertFalse(evaluation.wrappingThisCycle());
    assertFalse(evaluation.wrappingAfterUpdate());
  }

  @Test
  void clearsWrappingWhenNearCommandedTarget() {
    Turret.WrapEvaluation evaluation =
        Turret.evaluateWrappingState(
            Units.degreesToRadians(-351.0),
            Units.degreesToRadians(-353.0),
            true,
            MIN_RAD,
            MAX_RAD,
            ENTRY_MARGIN_RAD,
            EXIT_MARGIN_RAD,
            MIN_COMMAND_DELTA_RAD,
            NEAR_TARGET_DELTA_RAD);

    assertFalse(evaluation.wrappingThisCycle());
    assertFalse(evaluation.wrappingAfterUpdate());
  }

  @Test
  void clearsWrappingWhenAwayFromBoundaries() {
    Turret.WrapEvaluation evaluation =
        Turret.evaluateWrappingState(
            Units.degreesToRadians(-150.0),
            Units.degreesToRadians(-170.0),
            true,
            MIN_RAD,
            MAX_RAD,
            ENTRY_MARGIN_RAD,
            EXIT_MARGIN_RAD,
            MIN_COMMAND_DELTA_RAD,
            NEAR_TARGET_DELTA_RAD);

    assertFalse(evaluation.wrappingThisCycle());
    assertFalse(evaluation.wrappingAfterUpdate());
  }
}
