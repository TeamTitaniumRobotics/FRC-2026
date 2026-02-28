package org.teamtitanium.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Gains;

/***
 * Interface for Swerve Module Input/Output. Defines methods for updating inputs
 * and controlling the swerve module. Handles the data structure for swerve
 * module state.
 */
public interface SwerveModuleIO {
  /***
   * Data structure for Swerve Module IO Inputs.
   */
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public boolean turnConnected = false;
    public boolean turnCANcoderConnected = false;
    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /***
   * Updates the inputs for the swerve module.
   *
   * @param inputs The inputs to be updated.
   */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /***
   * Sets the drive motor to open loop control with the given output.
   *
   * @param output The output value for open loop control; either volts or amps.
   */
  public default void setDriveOpenLoop(double output) {}

  /***
   * Sets the turn motor to open loop control with the given output.
   *
   * @param output The output value for open loop control; either volts or amps.
   */
  public default void setTurnOpenLoop(double output) {}

  /***
   * Sets the drive motor to the given velocity in radians per second.
   *
   * @param velocityRadPerSec The target velocity in radians per second.
   */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /***
   * Sets the drive motor to the given velocity in radians per second with
   * feedforward current.
   *
   * @param velocityRadPerSec The target velocity in radians per second.
   * @param torqueFF  The feedforward torque to be applied.
   */
  public default void setDriveVelocity(double velocityRadPerSec, double torqueFF) {}

  /***
   * Sets the turn motor to the given position in radians.
   *
   * @param positionRad The target position in radians.
   */
  public default void setTurnPosition(double positionRad) {}

  /***
   * Updates the drive motor PID gains. Commonly used for tuning.
   *
   * @param gains The PID gains to be set.
   */
  public default void setDriveGains(Gains gains) {}

  /***
   * Updates the turn motor PID gains. Commonly used for tuning.
   *
   * @param gains The PID gains to be set.
   */
  public default void setTurnGains(Gains gains) {}

  /***
   * Sets the brake mode for both the drive and turn motors. Used for easily
   * moving the robot while disabled.
   *
   * @param enabled True to enable brake mode, false to disable.
   */
  public default void setBrakeMode(boolean enabled) {}
}
