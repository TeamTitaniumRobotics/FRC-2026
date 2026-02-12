package org.teamtitanium.subsystems.intake.rack;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class IntakeRackConstants {
  private IntakeRackConstants() {}

  // CAN IDs
  public static final int RACK_MOTOR_ID = 31;

  // Gearbox + mechanism geometry
  public static final double RACK_GEAR_RATIO = 15.0; // motor rotations per carriage rotation
  public static final double DRUM_DIAMETER_METERS = Units.inchesToMeters(1.5);
  public static final double DRUM_CIRCUMFERENCE_METERS = Math.PI * DRUM_DIAMETER_METERS;
  public static final double METERS_PER_MOTOR_ROTATION =
      DRUM_CIRCUMFERENCE_METERS / RACK_GEAR_RATIO;

  // Extension limits
  public static final double MIN_EXTENSION_METERS = Units.inchesToMeters(0.0);
  public static final double STOW_EXTENSION_METERS = Units.inchesToMeters(1.0);
  public static final double DEPLOY_EXTENSION_METERS = Units.inchesToMeters(13.0);
  public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(15.0);
  public static final double EXTENSION_TOLERANCE_METERS = Units.inchesToMeters(0.1);

  // Motion constraints (converted to motor rotations per second)
  private static final double MAX_LINEAR_VELOCITY_MPS = 0.75;
  private static final double MAX_LINEAR_ACCELERATION_MPS2 = 2.5;
  public static final Constraints RACK_CONSTRAINTS =
      new Constraints(
          MAX_LINEAR_VELOCITY_MPS / METERS_PER_MOTOR_ROTATION,
          MAX_LINEAR_ACCELERATION_MPS2 / METERS_PER_MOTOR_ROTATION);

  // Gains (Motion Magic position)
  public static final Gains RACK_GAINS = new Gains(8.0, 0.0, 0.1, 0.35, 0.0, 0.0, 0.05);

  // Current limits
  public static final double STATOR_CURRENT_LIMIT = 50.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;

  // Homing parameters
  public static final double HOMING_VOLTAGE_VOLTS = -2.5;
  public static final double HOMING_CURRENT_THRESHOLD_AMPS = 25.0;
  public static final double HOMING_VELOCITY_THRESHOLD_RPS = 0.05;
  public static final double HOMING_DEBOUNCE_TIME_SECS = 0.2;

  // Simulation
  public static final DCMotor RACK_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double RACK_MOMENT_OF_INERTIA = 0.025;

  public static double metersToMotorRotations(double extensionMeters) {
    return extensionMeters / METERS_PER_MOTOR_ROTATION;
  }

  public static double motorRotationsToMeters(double motorRotations) {
    return motorRotations * METERS_PER_MOTOR_ROTATION;
  }
}
