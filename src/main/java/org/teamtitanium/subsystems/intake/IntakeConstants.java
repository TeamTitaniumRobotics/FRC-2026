package org.teamtitanium.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import org.teamtitanium.subsystems.genericroller.GenericRoller.GenericRollerConstants;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class IntakeConstants {
  public static class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 30;
    public static final CANBus ROLLER_CAN_BUS = Constants.RIO_CAN_BUS;

    public static final boolean ROLLER_INVERTED = false;

    public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0.0);
    public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(16.0);

    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    public static final double ROLLER_GEAR_RATIO = 1.0;

    public static final Gains ROLLER_GAINS = new Gains(8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final Constraints ROLLER_CONSTRAINTS = new Constraints(24.0, 32.0);

    public static final GenericRollerConstants CONSTANTS =
        new GenericRollerConstants(
            ROLLER_MOTOR_ID,
            ROLLER_CAN_BUS,
            ROLLER_GEAR_RATIO,
            ROLLER_GAINS,
            ROLLER_CONSTRAINTS,
            STATOR_CURRENT_LIMIT,
            SUPPLY_CURRENT_LIMIT,
            ROLLER_INVERTED,
            true);
  }

  public static class RackConstants {
    // CAN IDs
    public static final int RACK_MOTOR_ID = 31;
    public static final CANBus RACK_CAN_BUS = Constants.RIO_CAN_BUS;

    // Clockwise (true) or counter-clockwise (false) positive
    public static final boolean RACK_INVERTED = false;

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 60.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;

    // Gearbox + mechanism geometry
    public static final double RACK_GEAR_RATIO = 15.0; // ratio from motor to rack
    public static final double GEAR_DIAMETER_METERS = Units.inchesToMeters(1.0);
    public static final double GEAR_CIRCUMFERENCE_METERS = Math.PI * GEAR_DIAMETER_METERS;
    public static final double METERS_PER_MOTOR_ROTATION =
        GEAR_CIRCUMFERENCE_METERS / RACK_GEAR_RATIO;

    // Extension limits
    public static final Distance MIN_EXTENSION = Inches.of(0.0);
    public static final Distance STOW_EXTENSION = Inches.of(0.0);
    public static final Distance DEPLOY_EXTENSION = Inches.of(10.0);
    public static final Distance MAX_EXTENSION = Inches.of(12.0);
    public static final Distance EXTENSION_TOLERANCE = Inches.of(0.1);

    // Gains
    public static final Gains RACK_GAINS = new Gains(8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Motion constraints (converted to motor rotations per second)
    private static final double MAX_LINEAR_VELOCITY_MPS = 0.75;
    private static final double MAX_LINEAR_ACCELERATION_MPS2 = 2.5;
    public static final Constraints RACK_CONSTRAINTS =
        new Constraints(
            MAX_LINEAR_VELOCITY_MPS / METERS_PER_MOTOR_ROTATION,
            MAX_LINEAR_ACCELERATION_MPS2 / METERS_PER_MOTOR_ROTATION);

    // Homing parameters
    public static final double HOMING_VOLTAGE_VOLTS = -2.5;
    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 25.0;
    public static final double HOMING_VELOCITY_THRESHOLD_RPS = 0.05;
    public static final double HOMING_DEBOUNCE_TIME_SECS = 0.2;

    // Simulation
    public static final DCMotor RACK_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double RACK_MOMENT_OF_INERTIA = 0.025;

    public static double metersToMotorRotations(double extensionMeters) {
      return extensionMeters / METERS_PER_MOTOR_ROTATION;
    }

    public static double motorRotationsToMeters(double motorRotations) {
      return motorRotations * METERS_PER_MOTOR_ROTATION;
    }
  }
}
