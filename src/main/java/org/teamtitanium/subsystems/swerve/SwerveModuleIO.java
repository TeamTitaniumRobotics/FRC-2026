package org.teamtitanium.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Gains;

public interface SwerveModuleIO {
  @AutoLog
  public class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public boolean turnConnected = false;
    public boolean turnCanCoderConnected = false;
    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;

    public double[] odometryTimestamps = new double[0];
    public double[] odometryDrivePositionsRad = new double[0];
    public double[] odometryTurnPositionsRad = new double[0];
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  public default void setDriveOpenLoop(double output) {}

  public default void setTurnOpenLoop(double output) {}

  public default void setDriveVelocity(double velocityRadPerSec) {}

  public default void setDriveVelocity(double velocityRadPerSec, double feedforwardVolts) {}

  public default void setTurnPosition(double positionRad) {}

  public default void setDriveGains(Gains gains) {}

  public default void setTurnGains(Gains gains) {}

  public default void setBrakeMode(boolean enabled) {}
}
