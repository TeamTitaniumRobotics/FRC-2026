package org.teamtitanium.utils;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final boolean tuningMode = false;
  public static final boolean disableHAL = false;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

  public record Constraints(double maxVelocity, double maxAcceleration) {}
}
