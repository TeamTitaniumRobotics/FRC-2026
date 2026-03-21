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
    public static final AngularVelocity AGITATE_VELOCITY = RotationsPerSecond.of(16.0);
    public static final AngularVelocity INTAKE_VELOCITY = RotationsPerSecond.of(36.0);
    public static final AngularVelocity EJECT_VELOCITY = RotationsPerSecond.of(-24.0);

    public static final double STATOR_CURRENT_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;

    public static final double ROLLER_GEAR_RATIO = (18.0 / 12.0);

    public static final Gains ROLLER_GAINS = new Gains(0.05, 0.0, 0.0, 0.195, 0.185, 0.0, 0.0);
    public static final Constraints ROLLER_CONSTRAINTS = new Constraints(36.0, 75.0);

    public static final DCMotor ROLLER_MOTOR_GEARBOX = DCMotor.getKrakenX44(1);
    public static final double ROLLER_MOI = 0.004;

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
            false);
  }

  public static class RackConstants {
    // CAN IDs
    public static final int RACK_MOTOR_ID = 31;
    public static final CANBus RACK_CAN_BUS =
        Constants.getMode() == Constants.Mode.REAL ? Constants.CANIVORE : Constants.RIO_CAN_BUS;

    // Clockwise (true) or counter-clockwise (false) positive
    public static final boolean RACK_INVERTED = true;

    // Current limits
    public static final double STATOR_CURRENT_LIMIT = 30.0;
    public static final double SUPPLY_CURRENT_LIMIT = 20.0;

    // Gearbox + mechanism geometry
    public static final double RACK_GEAR_RATIO =
        (24.0 / 18.0) * (38.0 / 12.0); // ratio from motor to rack
    public static final double GEAR_DIAMETER_METERS = Units.inchesToMeters(1.0);
    public static final double GEAR_CIRCUMFERENCE_METERS = Math.PI * GEAR_DIAMETER_METERS;

    // Extension limits
    public static final Distance MIN_EXTENSION = Inches.of(0.0);
    public static final Distance CLIMB_STOW_EXTENSION = Inches.of(0.0);
    public static final Distance STOW_EXTENSION = Inches.of(4.0);
    // public static final Distance DEPLOY_EXTENSION = Inches.of(11.5);
    public static final Distance DEPLOY_EXTENSION = Inches.of(9.5);
    public static final Distance MAX_EXTENSION = Inches.of(12.0);
    public static final Distance EXTENSION_TOLERANCE = Inches.of(2.0);

    public static final double STOW_STALL_CURRENT_THRESHOLD = 60.0;
    public static final double STOW_CURRENT_DEBOUNCE_TIME_SECS = 0.25;

    // Gains
    public static final Gains RACK_GAINS = new Gains(16.0, 0.0, 0.0, 0.275, 0.475, 0.0, 0.02);

    // Motion constraints (converted to motor rotations per second)
    private static final double MAX_LINEAR_VELOCITY_MPS = 1.2;
    private static final double MAX_LINEAR_ACCELERATION_MPS2 = 4.0;
    public static final Constraints RACK_CONSTRAINTS =
        new Constraints(
            MAX_LINEAR_VELOCITY_MPS / GEAR_CIRCUMFERENCE_METERS, // 15rps
            MAX_LINEAR_ACCELERATION_MPS2 / GEAR_CIRCUMFERENCE_METERS); // 50rps

    private static final double AGITATE_LINEAR_VELOCITY_MPS = 0.5;
    private static final double AGITATE_LINEAR_ACCELERATION_MPS2 = 2.0;
    public static final Constraints AGITATE_CONSTRAINTS =
        new Constraints(
            AGITATE_LINEAR_VELOCITY_MPS / GEAR_CIRCUMFERENCE_METERS,
            AGITATE_LINEAR_ACCELERATION_MPS2 / GEAR_CIRCUMFERENCE_METERS);

    // Homing parameters
    public static final double HOMING_VOLTAGE_VOLTS = -2.5;
    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 35.0;
    public static final double HOMING_VELOCITY_THRESHOLD_RPS = 0.05;
    public static final double HOMING_DEBOUNCE_TIME_SECS = 0.5;

    // Simulation
    public static final DCMotor RACK_GEARBOX = DCMotor.getKrakenX44Foc(1);
    public static final double RACK_MOMENT_OF_INERTIA = 0.025;

    public static double metersToMotorRotations(double extensionMeters) {
      return extensionMeters / GEAR_CIRCUMFERENCE_METERS;
    }

    public static double motorRotationsToMeters(double motorRotations) {
      return motorRotations * GEAR_CIRCUMFERENCE_METERS;
    }
  }
}
