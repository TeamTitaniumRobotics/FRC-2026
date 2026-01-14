package org.teamtitanium.utils;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;
  public static final boolean disableHAL = false;

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

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
    public Gains(double kP, double kI, double kD) {
      this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
    }
  }

  public record Constraints(double maxVelocity, double maxAcceleration) {}
}
