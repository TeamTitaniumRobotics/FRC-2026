package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class TurretConstants {
  public static final Angle TURRET_STOW_ANGLE = Rotations.of(0.0);

  // CAN IDs
  public static final int TURRET_MOTOR_ID = 15;
  public static final int TURRET_CANCODER_1_ID = 16;
  public static final int TURRET_CANCODER_2_ID = 17;

  // Physical Constants
  public static final double TURRET_GEAR_RATIO = (90.0 / 10.0) * (54.0 / 12.0); // 40.5:1 reduction
  public static final double TURRET_MOMENT_OF_INERTIA = 0.1; // kg*m^2

  // CANcoder Gear Ratios from Turret
  public static final double CANCODER_1_RATIO = 1.0; // Temp value
  public static final double CANCODER_2_RATIO = 3.0; // Temp value

  // Mechanical Limits
  public static final Angle MIN_ANGLE = Degrees.of(-180.0); // -180 degrees
  public static final Angle MAX_ANGLE = Degrees.of(180.0); // 180 degrees
  public static final Angle TURRET_RANGE = MAX_ANGLE.minus(MIN_ANGLE); // 360 degrees

  // Tolerance
  public static final double ANGLE_TOLERANCE_ROTS = 0.01; // ~3.6 degrees

  // Motion Magic Constraints
  public static final Constraints TURRET_CONSTRAINTS =
      new Constraints(2.0, 4.0); // Max velocity (rps), accel (rps^2)

  // PID Gains
  public static final Gains TURRET_GAINS = new Gains(4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;

  // Simulation
  public static final DCMotor TURRET_GEARBOX = DCMotor.getKrakenX44(1);
  public static final Transform3d TURRET_TO_ROBOT =
      new Transform3d(new Translation3d(-0.119063, 0.169863, 0.403377), Rotation3d.kZero);
}
