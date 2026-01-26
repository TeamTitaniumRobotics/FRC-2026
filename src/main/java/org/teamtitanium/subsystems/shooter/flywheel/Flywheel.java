package org.teamtitanium.subsystems.shooter.flywheel;

import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double targetVelocityRps = 0.0;

  /** Creates a new Flywheel subsystem. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    // Log additional data
    Logger.recordOutput("Flywheel/TargetVelocityRps", targetVelocityRps);
    Logger.recordOutput("Flywheel/TargetVelocityRPM", targetVelocityRps * 60.0);
    Logger.recordOutput("Flywheel/AverageVelocityRps", getAverageVelocityRps());
    Logger.recordOutput("Flywheel/AverageVelocityRPM", getAverageVelocityRps() * 60.0);
    Logger.recordOutput("Flywheel/AtTarget", atTarget());
  }

  /**
   * Sets the flywheel to a specific velocity using closed-loop control.
   *
   * @param velocityRps The target velocity in rotations per second
   */
  public void setVelocity(double velocityRps) {
    targetVelocityRps = velocityRps;
    io.setVelocity(velocityRps);
  }

  /**
   * Sets the flywheel to a specific velocity in RPM.
   *
   * @param velocityRPM The target velocity in RPM
   */
  public void setVelocityRPM(double velocityRPM) {
    setVelocity(velocityRPM / 60.0);
  }

  /**
   * Runs the flywheel at a specific voltage (open loop control).
   *
   * @param volts The voltage to apply
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the flywheel motors. */
  public void stop() {
    targetVelocityRps = 0.0;
    io.stop();
  }

  /**
   * Gets the average velocity of both flywheel motors.
   *
   * @return The average velocity in rotations per second
   */
  public double getAverageVelocityRps() {
    return (inputs.leftVelocityRps + inputs.rightVelocityRps) / 2.0;
  }

  /**
   * Gets the average velocity of both flywheel motors in RPM.
   *
   * @return The average velocity in RPM
   */
  public double getAverageVelocityRPM() {
    return getAverageVelocityRps() * 60.0;
  }

  /**
   * Gets the left motor velocity.
   *
   * @return The left motor velocity in rotations per second
   */
  public double getLeftVelocityRps() {
    return inputs.leftVelocityRps;
  }

  /**
   * Gets the right motor velocity.
   *
   * @return The right motor velocity in rotations per second
   */
  public double getRightVelocityRps() {
    return inputs.rightVelocityRps;
  }

  /**
   * Checks if the flywheel is at the target velocity.
   *
   * @return True if at target within tolerance
   */
  public boolean atTarget() {
    return Math.abs(getAverageVelocityRps() - targetVelocityRps) < VELOCITY_TOLERANCE_RPS;
  }

  /**
   * Sets the flywheel motors to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  // -------------------- Commands --------------------

  /**
   * Command to set the flywheel to a specific velocity.
   *
   * @param velocityRps The target velocity in rotations per second
   * @return A command that sets the flywheel velocity
   */
  public Command setVelocityCommand(double velocityRps) {
    return run(() -> setVelocity(velocityRps)).withName("Flywheel.SetVelocity");
  }

  /**
   * Command to set the flywheel to a specific velocity in RPM.
   *
   * @param velocityRPM The target velocity in RPM
   * @return A command that sets the flywheel velocity
   */
  public Command setVelocityRPMCommand(double velocityRPM) {
    return setVelocityCommand(velocityRPM / 60.0);
  }

  /**
   * Command to set the flywheel velocity and wait until at target.
   *
   * @param velocityRps The target velocity in rotations per second
   * @return A command that sets velocity and finishes when at target
   */
  public Command setVelocityAndWaitCommand(double velocityRps) {
    return run(() -> setVelocity(velocityRps))
        .until(this::atTarget)
        .withName("Flywheel.SetVelocityAndWait");
  }

  /**
   * Command to run the flywheel with variable velocity input.
   *
   * @param velocitySupplier Supplier for velocity in rotations per second
   * @return A command for variable flywheel control
   */
  public Command variableVelocityCommand(DoubleSupplier velocitySupplier) {
    return run(() -> setVelocity(velocitySupplier.getAsDouble()))
        .withName("Flywheel.VariableVelocity");
  }

  /**
   * Command to stop the flywheel.
   *
   * @return A command that stops the flywheel
   */
  public Command stopCommand() {
    return runOnce(this::stop).withName("Flywheel.Stop");
  }

  /**
   * Command to idle the flywheel at a low velocity.
   *
   * @param idleVelocityRps The idle velocity in rotations per second
   * @return A command that runs the flywheel at idle speed
   */
  public Command idleCommand(double idleVelocityRps) {
    return run(() -> setVelocity(idleVelocityRps)).withName("Flywheel.Idle");
  }
}
