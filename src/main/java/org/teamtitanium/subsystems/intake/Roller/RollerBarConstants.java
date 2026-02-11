package org.teamtitanium.subsystems.intake.Roller;

import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class RollerBarConstants {
  // CAN IDs
  public static final int ROLLER_MOTOR_ID = 0; // temp value

  // Physical Constants
  public static final double ROLLER_GEAR_RATIO = 1.0; // temp value
  public static final double ROLLER_MOMENT_OF_INERTIA = 0.001; // kg*m^2, temp value

  // Tolerance
  public static final double VELOCITY_TOLERANCE_RPS = 0.5; // temp

  // Motion Magic Constraints
  public static final Constraints ROLLER_CONSTRAINTS =
      new Constraints(5.0, 10.0); // Max velocity (rps), accel (rps^2)

  // PID Gains
  public static final Gains ROLLER_GAINS = new Gains(0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0);

  // Other Constants here
}
