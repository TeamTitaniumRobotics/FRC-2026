package org.teamtitanium.subsystems.shooter.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class TurretConstants {
  // CAN IDs
  public static final int TURRET_MOTOR_ID = 15;
  public static final int TURRET_CANCODER_1_ID = 16;
  public static final int TURRET_CANCODER_2_ID = 17;
  public static final String TURRET_CANBUS = "";

  // Physical Constants
  public static final double TURRET_GEAR_RATIO = 150.0 / 1.0; // 150:1 reduction
  public static final double TURRET_MOMENT_OF_INERTIA = 0.05; // kg*m^2

  // Mechanical Limits
  public static final double MIN_ANGLE_ROTS = -0.5; // -180 degrees
  public static final double MAX_ANGLE_ROTS = 0.5; // 180 degrees

  // Motion Magic Constraints
  public static final Constraints TURRET_MOTION_CONSTRAINTS =
      new Constraints(2.0, 4.0); // Max velocity (rps), accel (rps^2)

  // PID Gains
  public static final Gains TURRET_GAINS = new Gains(24.0, 0.0, 0.2);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;

  // Chinese Remainder Theorem CANcoder Configuration
  // CANcoder 1: Fine resolution (e.g., directly coupled or small ratio)
  public static final double CANCODER_1_RATIO = 1.0; // 1:1 with turret
  // CANcoder 2: Coarse resolution (e.g., through reduction)
  public static final double CANCODER_2_RATIO = 3.0; // 3:1 with turret

  // Simulation
  public static final DCMotor TURRET_GEARBOX = DCMotor.getFalcon500(1);
  
  // Tolerance
  public static final double POSITION_TOLERANCE_ROTS = 0.01; // ~3.6 degrees
}
