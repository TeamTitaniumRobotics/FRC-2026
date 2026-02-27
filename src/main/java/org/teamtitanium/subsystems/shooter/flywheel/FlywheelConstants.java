package org.teamtitanium.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class FlywheelConstants {
  // CAN IDs
  public static final int FLYWHEEL_LEFT_MOTOR_ID = 19;
  public static final int FLYWHEEL_RIGHT_MOTOR_ID = 20;
  public static final CANBus FLYWHEEL_CANBUS = Constants.RIO_CAN_BUS;

  public static final AngularVelocity IDLE_VELOCITY = RPM.of(1500.0);
  public static final AngularVelocity SHOOT_VELOCITY = RPM.of(3500);
  public static final AngularVelocity EJECT_VELOCITY = RPM.of(1000);

  // Physical Constants
  public static final double FLYWHEEL_GEAR_RATIO = (18.0 / 24.0); // 1:1.33 reduction
  public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.008; // kg*m^2

  // Tolerance
  public static final double VELOCITY_TOLERANCE_RPS = 2.0; // ~60 RPM tolerance

  // PID Gains (Velocity control)
  public static final Gains FLYWHEEL_GAINS = new Gains(0.3, 0.0, 0.0, 0.175, 0.095, 0.0, 0.01);
  public static final Constraints FLYWHEEL_CONSTRAINTS = new Constraints(100.0, 175.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 80.0; // High for flywheel spin-up
  public static final double SUPPLY_CURRENT_LIMIT = 60.0;

  // Simulation
  public static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(2); // Two motors
}
