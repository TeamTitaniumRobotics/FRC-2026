package org.teamtitanium.subsystems.shooter.hood;

import static org.teamtitanium.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double targetPositionRots = 0.0;

  /** Creates a new Hood subsystem. */
  public Hood(HoodIO io) {
    this.io = io;

    // Zero the hood using CANcoder on startup
    io.zeroWithCANcoder();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Log additional data
    Logger.recordOutput("Hood/TargetPositionRots", targetPositionRots);
    Logger.recordOutput("Hood/TargetPositionDeg", targetPositionRots * 360.0);
    Logger.recordOutput("Hood/AtTarget", atTarget());
  }

  /**
   * Sets the hood to a specific angle using Motion Magic.
   *
   * @param positionRots The target position in rotations
   */
  public void setPosition(double positionRots) {
    targetPositionRots = MathUtil.clamp(positionRots, MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
    io.setPosition(targetPositionRots);
  }

  /**
   * Sets the hood to a specific angle using Motion Magic.
   *
   * @param angle The target angle as a Rotation2d
   */
  public void setAngle(Rotation2d angle) {
    setPosition(angle.getRotations());
  }

  /**
   * Runs the hood at a specific voltage (open loop control).
   *
   * @param volts The voltage to apply
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the hood motor. */
  public void stop() {
    io.stop();
  }

  /**
   * Gets the current hood position.
   *
   * @return The current position in rotations
   */
  public double getPositionRots() {
    return inputs.positionRots;
  }

  /**
   * Gets the current hood angle.
   *
   * @return The current angle as a Rotation2d
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(inputs.positionRots);
  }

  /**
   * Gets the current hood velocity.
   *
   * @return The current velocity in rotations per second
   */
  public double getVelocityRps() {
    return inputs.velocityRps;
  }

  /**
   * Checks if the hood is at the target position.
   *
   * @return True if at target within tolerance
   */
  public boolean atTarget() {
    return Math.abs(inputs.positionRots - targetPositionRots) < POSITION_TOLERANCE_ROTS;
  }

  /** Zeros the hood using the CANcoder absolute position. */
  public void zeroWithCANcoder() {
    io.zeroWithCANcoder();
  }

  /**
   * Sets the hood to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  // -------------------- Commands --------------------

  /**
   * Command to set the hood to a specific angle.
   *
   * @param positionRots The target position in rotations
   * @return A command that sets the hood position
   */
  public Command setPositionCommand(double positionRots) {
    return run(() -> setPosition(positionRots)).withName("Hood.SetPosition");
  }

  /**
   * Command to set the hood to a specific angle.
   *
   * @param angle The target angle as a Rotation2d
   * @return A command that sets the hood angle
   */
  public Command setAngleCommand(Rotation2d angle) {
    return setPositionCommand(angle.getRotations());
  }

  /**
   * Command to set the hood to a specific angle and wait until at target.
   *
   * @param positionRots The target position in rotations
   * @return A command that sets the hood position and finishes when at target
   */
  public Command setPositionAndWaitCommand(double positionRots) {
    return run(() -> setPosition(positionRots))
        .until(this::atTarget)
        .withName("Hood.SetPositionAndWait");
  }

  /**
   * Command to run the hood with a joystick input (manual control).
   *
   * @param velocitySupplier Supplier for velocity in rotations per second
   * @return A command for manual hood control
   */
  public Command manualControlCommand(DoubleSupplier velocitySupplier) {
    return run(() -> {
          double velocity = velocitySupplier.getAsDouble();
          // Simple voltage control based on velocity request
          double volts = velocity * 3.0; // Adjust multiplier as needed
          setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
        })
        .withName("Hood.ManualControl");
  }

  /**
   * Command to zero the hood using CANcoder.
   *
   * @return A command that zeros the hood
   */
  public Command zeroCommand() {
    return runOnce(this::zeroWithCANcoder).withName("Hood.Zero");
  }

  /**
   * Command to stop the hood.
   *
   * @return A command that stops the hood
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Hood.Stop");
  }

  /**
   * Command to set hood to stow position (horizontal).
   *
   * @return A command that stows the hood
   */
  public Command stowCommand() {
    return setPositionCommand(MIN_ANGLE_ROTS).withName("Hood.Stow");
  }

  /**
   * Command to set hood angle based on distance calculation.
   *
   * @param angleSupplier Supplier for calculated angle
   * @return A command that aims the hood
   */
  public Command aimAtAngleCommand(DoubleSupplier angleSupplier) {
    return run(() -> setPosition(angleSupplier.getAsDouble())).withName("Hood.AimAtAngle");
  }
}
