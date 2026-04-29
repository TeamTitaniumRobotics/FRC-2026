package org.teamtitanium.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class HoodConstants {
  public static final Angle STOW_ANGLE = Degrees.of(0.0);
  public static final Angle EJECT_ANGLE = Degrees.of(0.0);

  // CAN IDs
  public static final int HOOD_MOTOR_ID = 18;
  public static final CANBus HOOD_CANBUS = Constants.RIO_CAN_BUS;

  // Physical Constants
  public static final double HOOD_GEAR_RATIO =
      (40.0 / 12.0) * (20.0 / 16.0) * (166.0 / 12.0); // ~57.639:1 reduction
  public static final double HOOD_MOMENT_OF_INERTIA = 0.02; // kg*m^2

  // Mechanical Limits
  public static final double MIN_ANGLE_ROTS = 0.0; // 0 degrees (horizontal)
  public static final double MAX_ANGLE_ROTS = 35.0 / 360.0; // 35 degrees

  // Tolerance
  public static final double ANGLE_TOLERANCE_ROTS = Units.degreesToRotations(3.0); // ~1.8 degrees

  // Motion Magic Constraints
  public static final Constraints HOOD_MOTION_CONSTRAINTS =
      new Constraints(1.5, 2.5); // Max velocity (rps), accel (rps^2)

  // High end: 0.475
  // Low end: 0.315

  // PID Gains
  public static final Gains HOOD_GAINS = new Gains(128.0, 0.0, 0.0, 0.08, 5.5, 0.395, 0.0);

  // Current Limits
  public static final double STATOR_CURRENT_LIMIT = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;

  // Zeroing
  public static final double ZERO_CURRENT_LIMIT = 35.0;
  public static final double ZERO_VELOCITY_LIMIT = 0.1;

  // Simulation
  public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX44(1);
}
