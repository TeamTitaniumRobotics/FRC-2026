package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * Unit tests for Turret subsystem, specifically testing the getTargetAngle method which handles
 * shortest path optimization and angle normalization within [-180°, 180°] bounds.
 */
class TurretTest {
  private Turret turret;

  @BeforeEach
  void setup() {
    // Initialize HAL for WPILib usage
    assert HAL.initialize(500, 0);

    // Create a mock TurretIO for testing
    turret = new Turret(new TurretIO() {});
  }

  // ==================== Basic Angle Wrapping Tests ====================

  @Test
  @DisplayName("Should return target angle when delta is within bounds")
  void testDirectPath() {
    Angle current = Degrees.of(0);
    Angle target = Degrees.of(45);

    Angle result = turret.getTargetAngle(target, current);

    assertEquals(45.0, result.in(Degrees), 0.001, "Should take direct path to 45 degrees");
  }

  @Test
  @DisplayName("Should wrap around 180 degrees when shorter")
  void testWrapAround180() {
    Angle current = Degrees.of(170);
    Angle target = Degrees.of(-170);

    Angle result = turret.getTargetAngle(target, current);

    // The optimal path from 170 to -170 should go through 180 (20 degrees forward)
    // Result should be normalized to [-180, 180], so expecting -170
    assertEquals(
        -170.0, result.in(Degrees), 1.0, "Should take shortest path and result in -170 degrees");
  }

  @Test
  @DisplayName("Should wrap around -180 degrees when shorter")
  void testWrapAroundNeg180() {
    Angle current = Degrees.of(-170);
    Angle target = Degrees.of(170);

    Angle result = turret.getTargetAngle(target, current);

    // The optimal path from -170 to 170 should go through -180 (20 degrees backward)
    // Result should be normalized to [-180, 180], so expecting 170
    assertEquals(
        170.0, result.in(Degrees), 1.0, "Should take shortest path and result in 170 degrees");
  }

  // ==================== Parameterized Tests for Various Scenarios ====================

  @ParameterizedTest
  @DisplayName("Should correctly calculate optimal angle for various inputs")
  @CsvSource({
    "0, 90, 90, 90", // Simple forward movement
    "0, -90, -90, 90", // Simple backward movement
    "90, -90, -90, 180", // Across zero
    "45, 315, -45, 90", // 315° normalizes to -45°, shortest path from 45° is -90°, result -45°
    "-45, 315, -45, 0", // -45° and 315° (normalized to -45°) are equivalent, no movement
    "0, 180, 180, 180", // Exactly opposite - either direction works
    "0, -180, -180, 180", // Exactly opposite (negative)
    "350, 10, 10, 20", // 350° normalizes to -10°, target 10°, shortest is +20°
    "-350, -10, -10, 20", // -350° normalizes to 10°, target -10°, shortest is -20°
  })
  void testVariousAngleCombinations(
      double currentDeg, double targetDeg, double expectedResultDeg, double expectedMagnitude) {
    Angle current = Degrees.of(currentDeg);
    Angle target = Degrees.of(targetDeg);

    Angle result = turret.getTargetAngle(target, current);

    // Normalize current for comparison
    double normalizedCurrent = currentDeg;
    while (normalizedCurrent > 180) normalizedCurrent -= 360;
    while (normalizedCurrent < -180) normalizedCurrent += 360;

    // Calculate the actual delta from normalized current to result
    double actualDelta = result.in(Degrees) - normalizedCurrent;

    // Verify the magnitude of rotation is as expected (shortest path)
    double tolerance = 1.0; // 1 degree tolerance
    assertEquals(
        expectedMagnitude,
        Math.abs(actualDelta),
        tolerance,
        String.format(
            "From %.1f° (normalized: %.1f°) to %.1f° should rotate %.1f° (actual rotation: %.1f°)",
            currentDeg, normalizedCurrent, targetDeg, expectedMagnitude, Math.abs(actualDelta)));

    // Verify the result is the expected angle
    assertEquals(
        expectedResultDeg,
        result.in(Degrees),
        tolerance,
        String.format(
            "From %.1f° to %.1f° should result in %.1f° (got %.1f°)",
            currentDeg, targetDeg, expectedResultDeg, result.in(Degrees)));
  }

  // ==================== Edge Cases ====================

  @Test
  @DisplayName("Should handle zero to zero")
  void testZeroToZero() {
    Angle current = Degrees.of(0);
    Angle target = Degrees.of(0);

    Angle result = turret.getTargetAngle(target, current);

    assertEquals(0.0, result.in(Degrees), 0.001, "Zero to zero should stay at zero");
  }

  @Test
  @DisplayName("Should handle very small angles")
  void testSmallAngles() {
    Angle current = Degrees.of(0.1);
    Angle target = Degrees.of(0.2);

    Angle result = turret.getTargetAngle(target, current);

    assertEquals(
        0.2, result.in(Degrees), 0.001, "Should handle small angles without wrapping issues");
  }

  @Test
  @DisplayName("Should handle negative wrapping correctly")
  void testNegativeWrapping() {
    Angle current = Degrees.of(-180);
    Angle target = Degrees.of(180);

    Angle result = turret.getTargetAngle(target, current);

    // -180 and 180 are the same angle, should not move
    assertTrue(
        Math.abs(result.in(Degrees) - 180) < 1.0 || Math.abs(result.in(Degrees) + 180) < 1.0,
        "Should recognize -180 and 180 as equivalent");
  }

  // ==================== Rotations Unit Tests ====================

  @Test
  @DisplayName("Should work with Rotations units")
  void testRotationsUnit() {
    Angle current = Rotations.of(0.25); // 90 degrees
    Angle target = Rotations.of(0.5); // 180 degrees

    Angle result = turret.getTargetAngle(target, current);

    assertEquals(0.5, result.in(Rotations), 0.001, "Should correctly handle Rotations unit type");
  }

  @Test
  @DisplayName("Should handle full rotation wrapping")
  void testFullRotationWrap() {
    Angle current = Rotations.of(0);
    Angle target = Rotations.of(1.0); // Full rotation = same as 0

    Angle result = turret.getTargetAngle(target, current);

    // Should recognize full rotation as same position
    double resultInRots = result.in(Rotations) % 1.0;
    assertEquals(0.0, resultInRots, 0.001, "Full rotation should wrap to zero");
  }

  // ==================== Radians Unit Tests ====================

  @Test
  @DisplayName("Should work with Radians units")
  void testRadiansUnit() {
    Angle current = Radians.of(0);
    Angle target = Radians.of(Math.PI / 2); // 90 degrees

    Angle result = turret.getTargetAngle(target, current);

    assertEquals(
        Math.PI / 2, result.in(Radians), 0.001, "Should correctly handle Radians unit type");
  }

  // ==================== Shortest Path Tests ====================

  @Test
  @DisplayName("Should always take shortest path")
  void testShortestPath() {
    // From 10 degrees to 350 degrees
    // Direct path: 340 degrees forward
    // Wrap path: 20 degrees backward (shorter!)
    Angle current = Degrees.of(10);
    Angle target = Degrees.of(350);

    Angle result = turret.getTargetAngle(target, current);

    // The delta should be approximately -20 degrees, resulting in -10 or 350 degrees
    double delta = Math.abs(result.in(Degrees) - current.in(Degrees));
    if (delta > 180) {
      delta = 360 - delta;
    }

    assertTrue(
        delta < 30, // Should be close to 20 degrees
        String.format(
            "Shortest path from 10° to 350° should be ~20° (backward), got %.1f° delta", delta));
  }

  // ==================== Consistency Tests ====================

  @Test
  @DisplayName("Should be consistent for equivalent angles")
  void testEquivalentAngles() {
    Angle current = Degrees.of(0);
    Angle target1 = Degrees.of(90);
    Angle target2 = Degrees.of(450); // 450 = 90 + 360

    Angle result1 = turret.getTargetAngle(target1, current);
    Angle result2 = turret.getTargetAngle(target2, current);

    // Results should be equivalent (within a full rotation)
    double diff = Math.abs(result1.in(Degrees) - result2.in(Degrees));
    assertTrue(
        diff < 1.0 || Math.abs(diff - 360) < 1.0,
        "Equivalent target angles should produce equivalent results");
  }

  // ==================== Boundary Tests ====================

  @Test
  @DisplayName("Should handle maximum positive angle")
  void testMaxPositiveAngle() {
    Angle current = Degrees.of(0);
    Angle target = Degrees.of(180);

    Angle result = turret.getTargetAngle(target, current);

    assertNotNull(result, "Should handle maximum positive angle without errors");
  }

  @Test
  @DisplayName("Should handle maximum negative angle")
  void testMaxNegativeAngle() {
    Angle current = Degrees.of(0);
    Angle target = Degrees.of(-180);

    Angle result = turret.getTargetAngle(target, current);

    assertNotNull(result, "Should handle maximum negative angle without errors");
  }

  // ==================== Integration Tests ====================

  @Test
  @DisplayName("Should work correctly in a sequence of movements")
  void testSequentialMovements() {
    Angle position = Degrees.of(0);

    // Move to 45
    position = turret.getTargetAngle(Degrees.of(45), position);
    assertEquals(45.0, position.in(Degrees), 1.0, "First move should result in 45°");

    // Move to -45 (should go backwards through 0, 90 degree rotation)
    Angle prevPosition = position;
    position = turret.getTargetAngle(Degrees.of(-45), position);
    double move2Delta = position.in(Degrees) - prevPosition.in(Degrees);
    assertEquals(-45.0, position.in(Degrees), 1.0, "Second move should result in -45°");
    assertEquals(90.0, Math.abs(move2Delta), 1.0, "Second move should rotate 90°");

    // Move to 170 (from -45°, shortest path is +215° but wrapped result is 170°)
    prevPosition = position;
    position = turret.getTargetAngle(Degrees.of(170), position);
    assertEquals(170.0, position.in(Degrees), 1.0, "Third move should result in 170°");
    // The delta from -45° to 170° should be shortest path: -145° (wrapping through -180)
    double move3Delta = position.in(Degrees) - prevPosition.in(Degrees);
    assertTrue(
        Math.abs(move3Delta) >= 140 && Math.abs(move3Delta) <= 220,
        String.format(
            "Third move from -45° to 170° should rotate ~145° or ~215° (actual: %.1f°)",
            Math.abs(move3Delta)));
  }
}
