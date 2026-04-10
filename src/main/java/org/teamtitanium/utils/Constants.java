package org.teamtitanium.utils;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final double simLoopPeriodSecs = 0.002;
  public static final boolean tuningMode = true;
  public static final boolean disableHAL = false;

  public static final Distance BUMPER_HALF_LENGTH =
      Meters.of(TunerConstants.FrontLeft.LocationX).plus(Inches.of(3.5));
  public static final Distance BUMPER_HALF_WIDTH =
      Meters.of(TunerConstants.FrontLeft.LocationY).plus(Inches.of(3.5));

  private static final Mode simMode = Mode.SIM;

  public static Mode getMode() {
    return RobotBase.isReal() ? Mode.REAL : simMode;
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final CANBus RIO_CAN_BUS = CANBus.roboRIO();
  public static final CANBus CANIVORE = TunerConstants.kCANBus;

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
    public Gains(double kP, double kI, double kD) {
      this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
    }
  }

  public record Constraints(double maxVelocity, double maxAcceleration, double kJerk) {
    public Constraints(double maxVelocity, double maxAcceleration) {
      this(maxVelocity, maxAcceleration, 0.0);
    }
  }
}
