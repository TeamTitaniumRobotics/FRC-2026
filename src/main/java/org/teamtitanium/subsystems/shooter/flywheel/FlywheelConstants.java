package org.teamtitanium.subsystems.shooter.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import org.teamtitanium.utils.Constants.Gains;

public class FlywheelConstants {
  // CAN IDs
  public static final int FLYWHEEL_LEFT_MOTOR_ID = 20;
  public static final int FLYWHEEL_RIGHT_MOTOR_ID = 21;
  public static final String FLYWHEEL_CANBUS = "";

  // Physical Constants
  public static final double FLYWHEEL_GEAR_RATIO = 1.0 / 1.0; // Direct drive
  public static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.008; // kg*m^2

  // PID Gains (Velocity control)
  public static final Gains FLYWHEEL_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.12, 0.0, 0.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 80.0; // High for flywheel spin-up
  public static final double SUPPLY_CURRENT_LIMIT = 60.0;

  // Simulation
  public static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(2); // Two motors

  // Tolerance
  public static final double VELOCITY_TOLERANCE_RPS = 1.0; // ~60 RPM tolerance
}
