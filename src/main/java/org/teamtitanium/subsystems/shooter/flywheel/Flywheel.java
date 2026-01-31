package org.teamtitanium.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {
  // Real-time tunable PID gains
  private final LoggedTunableNumber flywheelkP =
      new LoggedTunableNumber("Flywheel/kP", FLYWHEEL_GAINS.kP());
  private final LoggedTunableNumber flywheelkI =
      new LoggedTunableNumber("Flywheel/kI", FLYWHEEL_GAINS.kI());
  private final LoggedTunableNumber flywheelkD =
      new LoggedTunableNumber("Flywheel/kD", FLYWHEEL_GAINS.kD());
  private final LoggedTunableNumber flywheelkS =
      new LoggedTunableNumber("Flywheel/kS", FLYWHEEL_GAINS.kS());
  private final LoggedTunableNumber flywheelkV =
      new LoggedTunableNumber("Flywheel/kV", FLYWHEEL_GAINS.kV());
  private final LoggedTunableNumber flywheelkA =
      new LoggedTunableNumber("Flywheel/kA", FLYWHEEL_GAINS.kA());

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double targetVelocityRps = 0.0;
  private final Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.velocityRps - targetVelocityRps) < VELOCITY_TOLERANCE_RPS);

  /** Creates a new Flywheel subsystem. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (flywheelkP.hasChanged(hashCode())
        || flywheelkI.hasChanged(hashCode())
        || flywheelkD.hasChanged(hashCode())
        || flywheelkS.hasChanged(hashCode())
        || flywheelkV.hasChanged(hashCode())
        || flywheelkA.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              flywheelkP.get(),
              flywheelkI.get(),
              flywheelkD.get(),
              flywheelkS.get(),
              flywheelkV.get(),
              0.0,
              flywheelkA.get()));
    }

    LoggedTracer.record("Flywheel");
  }

  /**
   * Sets the flywheel to a velocity supplier
   *
   * @param velocityRps target supplier velocity for the flywheel
   * @return A command that repeatedly sets the flywheel to a velocity
   */
  public Command setVelocity(DoubleSupplier velocityRps) {
    return run(() -> {
          targetVelocityRps = velocityRps.getAsDouble();
          io.setVelocity(velocityRps.getAsDouble());
        })
        .withName("Flywheel.SetVelocity");
  }

  /**
   * Sets the flywheel to a velocity
   *
   * @param velocityRps target velocity for the flywheel
   * @return A command that repeatedly sets the flywheel to a velocity
   */
  public Command setVelocity(double velocityRps) {
    return setVelocity(() -> velocityRps);
  }

  /**
   * Runs the flywheel at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the flywheel at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).withName("Flywheel.SetVoltage");
  }

  /**
   * Runs the flywheel at a given voltage
   *
   * @param voltage target voltage
   * @return A command that repeatedly runs the flywheel at a voltage
   */
  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  /**
   * Gets the average velocity of both flywheel motors.
   *
   * @return The average flywheel velocity
   */
  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRps);
  }

  /**
   * Checks if the flywheel is at the target velocity.
   *
   * @return A trigger that is true if at target within tolerance
   */
  @AutoLogOutput(key = "Flywheel/AtSetpoint")
  public Trigger atSetpoint() {
    return atSetpoint;
  }

  /**
   * Sets the flywheel motors to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  /** Stops the flywheel motors. */
  public void stop() {
    targetVelocityRps = 0.0;
    io.setVoltage(0.0);
  }
}
