package org.teamtitanium.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class HoodConstants {
  // CAN IDs
  public static final int HOOD_MOTOR_ID = 18;
  public static final int HOOD_CANCODER_ID = 19;
  public static final String HOOD_CANBUS = "";

  // Physical Constants
  public static final double HOOD_GEAR_RATIO = 100.0 / 1.0; // 100:1 reduction
  public static final double HOOD_MOMENT_OF_INERTIA = 0.02; // kg*m^2

  // Mechanical Limits (in rotations)
  public static final double MIN_ANGLE_ROTS = 0.0; // 0 degrees (horizontal)
  public static final double MAX_ANGLE_ROTS = 0.25; // 90 degrees (vertical)

  // Motion Magic Constraints
  public static final Constraints HOOD_MOTION_CONSTRAINTS =
      new Constraints(1.5, 3.0); // Max velocity (rps), accel (rps^2)

  // PID Gains
  public static final Gains HOOD_GAINS = new Gains(20.0, 0.0, 0.15, 0.0, 0.0, 0.3, 0.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT = 30.0;

  // CANcoder offset (absolute position when hood is at 0 degrees)
  public static final double CANCODER_OFFSET_ROTS = 0.0;

  // Simulation
  public static final DCMotor HOOD_GEARBOX = DCMotor.getFalcon500(1);

  // Tolerance
  public static final double POSITION_TOLERANCE_ROTS = 0.005; // ~1.8 degrees
}
