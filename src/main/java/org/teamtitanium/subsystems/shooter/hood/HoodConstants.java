package org.teamtitanium.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class HoodConstants {
  // CAN IDs
  public static final int HOOD_MOTOR_ID = 18;

  // Physical Constants
  public static final double HOOD_GEAR_RATIO = 100.0; // 100:1 reduction
  public static final double HOOD_MOMENT_OF_INERTIA = 0.02; // kg*m^2

  // Mechanical Limits
  public static final double MIN_ANGLE_ROTS = 0.0; // 0 degrees (horizontal)
  public static final double MAX_ANGLE_ROTS = 0.25; // 90 degrees (vertical)

  // Tolerance
  public static final double POSITION_TOLERANCE_ROTS = 0.005; // ~1.8 degrees

  // Motion Magic Constraints
  public static final Constraints HOOD_MOTION_CONSTRAINTS =
      new Constraints(1.5, 3.0); // Max velocity (rps), accel (rps^2)

  // PID Gains
  public static final Gains HOOD_GAINS = new Gains(20.0, 0.0, 0.15, 0.0, 0.0, 0.3, 0.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT = 30.0;

  // Simulation
  public static final DCMotor HOOD_GEARBOX = DCMotor.getFalcon500(1);
}
