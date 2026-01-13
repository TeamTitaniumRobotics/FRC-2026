package org.teamtitanium.subsystems.swerve;

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
  public class SwerveModuleIOInputs {
    public SwerveModuleIOData swerveModuleData =
        new SwerveModuleIOData(
            false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public double[] odometryDrivePositionsRad = new double[0];
    public double[] odometryTurnPositionsRad = new double[0];
  }

  /***
   * Data structure for Swerve Module IO Data. Storing the data in a record can
   * improve performance and immutability.
   */
  public record SwerveModuleIOData(
      boolean driveConnected,
      double drivePositionRad,
      double driveVelocityRadPerSec,
      double driveAppliedVolts,
      double driveSupplyCurrentAmps,
      double driveTorqueCurrentAmps,
      double driveTempCelcius,
      boolean turnConnected,
      boolean turnCanCoderConnected,
      double turnAbsolutePositionRad,
      double turnPositionRad,
      double turnVelocityRadPerSec,
      double turnAppliedVolts,
      double turnSupplyCurrentAmps,
      double turnTorqueCurrentAmps,
      double turnTempCelcius) {}

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
   * feedforward voltage.
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
