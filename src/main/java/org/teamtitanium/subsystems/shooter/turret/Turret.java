package org.teamtitanium.subsystems.shooter.turret;

import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private double targetPositionRots = 0.0;

  /** Creates a new Turret subsystem. */
  public Turret(TurretIO io) {
    this.io = io;

    // Zero the turret using CANcoders on startup if available
    // zeroTurret(); TODO: Implement CRT zeroing method
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  /**
   * Sets the turret to a specific angle using Motion Magic.
   *
   * @param positionRots The target position in rotations
   */
  public void setPosition(double positionRots) {
    targetPositionRots = MathUtil.clamp(positionRots, MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
    io.setPosition(targetPositionRots);
  }

  /**
   * Sets the turret to a specific angle using Motion Magic.
   *
   * @param angle The target angle as a Rotation2d
   */
  public void setAngle(Rotation2d angle) {
    setPosition(angle.getRotations());
  }

  /**
   * Runs the turret at a specific voltage (open loop control).
   *
   * @param volts The voltage to apply
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the turret motor. */
  public void stop() {
    io.stop();
  }

  /**
   * Gets the current turret position.
   *
   * @return The current position in rotations
   */
  public double getPositionRots() {
    return inputs.positionRots;
  }

  /**
   * Gets the current turret angle.
   *
   * @return The current angle as a Rotation2d
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(inputs.positionRots);
  }

  /**
   * Gets the current turret velocity.
   *
   * @return The current velocity in rotations per second
   */
  public double getVelocityRps() {
    return inputs.velocityRps;
  }

  /**
   * Checks if the turret is at the target position.
   *
   * @return True if at target within tolerance
   */
  public boolean atTarget() {
    return Math.abs(inputs.positionRots - targetPositionRots) < POSITION_TOLERANCE_ROTS;
  }

  /** Zeros the turret using the CANcoders and Chinese Remainder Theorem. */
  public void zeroWithCANcoders() {
    io.zeroWithCANcoders();
  }

  /**
   * Sets the turret to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  // -------------------- Commands --------------------

  /**
   * Command to set the turret to a specific angle.
   *
   * @param positionRots The target position in rotations
   * @return A command that sets the turret position
   */
  public Command setPositionCommand(double positionRots) {
    return run(() -> setPosition(positionRots)).withName("Turret.SetPosition");
  }

  /**
   * Command to set the turret to a specific angle.
   *
   * @param angle The target angle as a Rotation2d
   * @return A command that sets the turret angle
   */
  public Command setAngleCommand(Rotation2d angle) {
    return setPositionCommand(angle.getRotations());
  }

  /**
   * Command to set the turret to a specific angle and wait until at target.
   *
   * @param positionRots The target position in rotations
   * @return A command that sets the turret position and finishes when at target
   */
  public Command setPositionAndWaitCommand(double positionRots) {
    return run(() -> setPosition(positionRots))
        .until(this::atTarget)
        .withName("Turret.SetPositionAndWait");
  }

  /**
   * Command to run the turret with a joystick input (manual control).
   *
   * @param velocitySupplier Supplier for velocity in rotations per second
   * @return A command for manual turret control
   */
  public Command manualControlCommand(DoubleSupplier velocitySupplier) {
    return run(() -> {
          double velocity = velocitySupplier.getAsDouble();
          // Simple voltage control based on velocity request
          double volts = velocity * 2.0; // Adjust multiplier as needed
          setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
        })
        .withName("Turret.ManualControl");
  }

  /**
   * Command to zero the turret using CANcoders.
   *
   * @return A command that zeros the turret
   */
  public Command zeroCommand() {
    return runOnce(this::zeroWithCANcoders).withName("Turret.Zero");
  }

  /**
   * Command to stop the turret.
   *
   * @return A command that stops the turret
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Turret.Stop");
  }

  /**
   * Command to aim at a specific field position (example for future implementation).
   *
   * @param fieldAngle The target field angle
   * @return A command that aims the turret
   */
  public Command aimAtAngleCommand(DoubleSupplier fieldAngle) {
    return run(() -> setPosition(fieldAngle.getAsDouble())).withName("Turret.AimAtAngle");
  }
}
