package org.teamtitanium.subsystems.shooter.backroller;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class BackRollerConstants {
  // CAN IDs
  public static final int BACK_ROLLER_MOTOR_ID = 21;
  public static final CANBus BACK_ROLLER_CANBUS = Constants.RIO_CAN_BUS;

  public static final AngularVelocity MAX_VELOCITY = RPM.of(9300.0);

  // Physical Constants
  public static final double BACK_ROLLER_GEAR_RATIO = (16.0 / 24.0); // 1:1.5 reduction
  public static final boolean BACK_ROLLER_INVERTED = false;
  public static final double BACK_ROLLER_MOMENT_OF_INERTIA = 0.008; // kg*m^2

  public static final double BACK_ROLLER_CIRCUMFERENCE = Units.inchesToMeters(1.625 * Math.PI);
  public static final double FLYWHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0 * Math.PI);
  public static final double WHEEL_RATIO = BACK_ROLLER_CIRCUMFERENCE / FLYWHEEL_CIRCUMFERENCE;

  // Tolerance
  public static final double VELOCITY_TOLERANCE_RPS = 6.0; // ~60 RPM tolerance
  public static final double VELOCITY_GAIN_TOLERANCE_RPS = 2.0;

  // PID Gains (Velocity control)
  public static final Gains BACK_ROLLER_GAINS =
      new Gains(0.3, 0.0, 0.0, 0.15548, 0.062793, 0.0, 0.0020985);
  public static final Constraints BACK_ROLLER_CONSTRAINTS =
      new Constraints(MAX_VELOCITY.in(RotationsPerSecond), 175.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 80.0;
  public static final double SUPPLY_CURRENT_LIMIT = 60.0;

  // Simulation
  public static final DCMotor BACK_ROLLER_GEARBOX = DCMotor.getKrakenX44(1);
}
