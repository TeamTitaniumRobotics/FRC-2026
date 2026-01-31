package org.teamtitanium.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
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

  public enum FlywheelState {
    IDLE(() -> RPM.of(0.0)),
    SCORE(() -> RobotState.getInstance().getFlywheelSetpoint()),
    PASS(() -> RobotState.getInstance().getFlywheelSetpoint()),
    EJECT(() -> RPM.of(30.0));

    private final Supplier<AngularVelocity> targetVelocitySupplier;

    private FlywheelState(Supplier<AngularVelocity> targetVelocitySupplier) {
      this.targetVelocitySupplier = targetVelocitySupplier;
    }

    public AngularVelocity getTargetVelocityRps() {
      return targetVelocitySupplier.get();
    }
  }

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput(key = "Flywheel/State")
  @Getter
  @Setter
  private FlywheelState currentState = FlywheelState.IDLE;

  @AutoLogOutput(key = "Flywheel/ManualOverride")
  @Getter
  @Setter
  private boolean manualOverride = false;

  @AutoLogOutput(key = "Flywheel/TargetVelocityRps")
  private double targetVelocityRps = 0.0;

  private final Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.velocityRps - targetVelocityRps) < VELOCITY_TOLERANCE_RPS);

  /** Creates a new Flywheel subsystem. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    setDefaultCommand(setStateVelocity());
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
   * Sets the flywheel to the velocity defined by the current state
   *
   * @return A command that repeatedly sets the flywheel to the current state's velocity
   */
  private Command setStateVelocity() {
    return run(() -> {
          if (!manualOverride) {
            targetVelocityRps = currentState.getTargetVelocityRps().in(RotationsPerSecond);
            io.setVelocity(targetVelocityRps);
          }
        })
        .withName("Flywheel.SetStateVelocity");
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

  /**
   * Manual control command for the flywheel. Bypasses state-based control.
   *
   * @param voltageSupplier Voltage supplier for manual control
   * @return A command for manual flywheel control
   */
  public Command manualControl(DoubleSupplier voltageSupplier) {
    return run(() -> {
          setManualOverride(true);
          io.setVoltage(voltageSupplier.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
        .withName("Flywheel.ManualControl");
  }
}
