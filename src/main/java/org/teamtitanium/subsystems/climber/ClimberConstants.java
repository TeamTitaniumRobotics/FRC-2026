package org.teamtitanium.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class ClimberConstants {
  public static final int CLIMBER_MOTOR_ID = 40;
  public static final CANBus CLIMBER_CAN_BUS = Constants.CANIVORE;

  public static final boolean CLIMBER_INVERTED = false;

  public static final double CLIMBER_STATOR_CURRENT_LIMIT = 60.0;
  public static final double CLIMBER_SUPPLY_CURRENT_LIMIT = 40.0;

  public static final double CLIMBER_GEAR_RATIO = 16.0;
  private static final double SPROCKET_PD_METERS = Units.inchesToMeters(0.9659);
  private static final double SPROCKET_CIRCUMFERENCE_METERS = Math.PI * SPROCKET_PD_METERS;

  public static final Distance CLIMBER_MAX_EXTENSION = Inches.of(24.0);
  public static final Distance CLIMBER_MIN_EXTENSION = Inches.of(0.0);
  public static final Distance CLIMBER_UP_EXTENSION = Inches.of(12.0);
  public static final Distance CLIMBER_DOWN_EXTENSION = Inches.of(0.0);
  public static final Distance CLIMBER_STOW_EXTENSION = Inches.of(0.0);
  public static final Distance CLIMBER_TOLERANCE = Inches.of(0.5);

  public static final Gains CLIMBER_GAINS_SLOT_0 = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  public static final Gains CLIMBER_GAINS_SLOT_1 = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final double MAX_VERTICAL_VELOCITY_MPS = 1.2;
  public static final double MAX_VERTICAL_ACCELERATION_MPS2 = 4.0;
  public static final Constraints CLIMBER_CONSTRAINTS =
      new Constraints(
          MAX_VERTICAL_VELOCITY_MPS / SPROCKET_CIRCUMFERENCE_METERS,
          MAX_VERTICAL_ACCELERATION_MPS2 / SPROCKET_CIRCUMFERENCE_METERS);

  public static final double HOMING_CURRENT_AMPS = -10.0;
  public static final double HOMING_CURRENT_THRESHOLD_AMPS = 40.0;
  public static final double HOMING_VELOCITY_THRESHOLD_RPS = 0.05;
  public static final double HOMING_DEBOUNCE_TIME_SECS = 0.1;

  public static final DCMotor CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double CLIMBER_MOMENT_OF_INERTIA = 0.025;

  public static double metersToMotorRotations(double extensionMeters) {
    return extensionMeters / SPROCKET_CIRCUMFERENCE_METERS;
  }

  public static double motorRotationsToMeters(double motorRotations) {
    return motorRotations * SPROCKET_CIRCUMFERENCE_METERS;
  }
}
