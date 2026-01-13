package org.teamtitanium.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public GyroIOData gyroData = new GyroIOData(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public double[] odometryYawTimestamps = new double[0];
    public double[] odometryYawPositionsRads = new double[0];
  }

  public record GyroIOData(
    boolean connected,
    double yawPositionRads,
    double yawVelocityRadPerSec,
    double pitchPositionRads,
    double pitchVelocityRadPerSec,
    double rollPositionRads,
    double rollVelocityRadPerSec
  ) {
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
