package org.teamtitanium.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import org.teamtitanium.utils.Constants.Gains;

public class FlywheelConstants {
  // CAN IDs
  public static final int FLYWHEEL_LEFT_MOTOR_ID = 20;
  public static final int FLYWHEEL_RIGHT_MOTOR_ID = 21;
  public static final String FLYWHEEL_CANBUS = "";

  public static final AngularVelocity IDLE_VELOCITY = RPM.of(1500);
  public static final AngularVelocity SHOOT_VELOCITY = RPM.of(3000);
  public static final AngularVelocity EJECT_VELOCITY = RPM.of(1000);

  // Physical Constants
  public static final double FLYWHEEL_GEAR_RATIO = 0.5; // 1:2 reduction
  public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.008; // kg*m^2

  // Tolerance
  public static final double VELOCITY_TOLERANCE_RPS = 1.0; // ~60 RPM tolerance

  // PID Gains (Velocity control)
  public static final Gains FLYWHEEL_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.12, 0.0, 0.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 80.0; // High for flywheel spin-up
  public static final double SUPPLY_CURRENT_LIMIT = 60.0;

  // Simulation
  public static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(2); // Two motors
}
