package org.teamtitanium.subsystems.swerve;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPositionRads = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double pitchPositionRads = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double rollPositionRads = 0.0;
    public double rollVelocityRadPerSec = 0.0;

    public double[] odometryYawTimestamps = new double[0];
    public double[] odometryYawPositionsRads = new double[0];
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void setGyroAngle(Angle angle) {}
}
