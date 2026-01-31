package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
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
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class Turret extends SubsystemBase {
  public enum TurretState {
    STOW(() -> Rotations.of(0.0)),
    TRACK(() -> RobotState.getInstance().getTurretSetpoint()),
    EJECT(() -> Rotations.of(-0.25));

    private final Supplier<Angle> targetPositionSupplier;

    private TurretState(Supplier<Angle> targetPositionSupplier) {
      this.targetPositionSupplier = targetPositionSupplier;
    }

    public Angle getTargetPosition() {
      return targetPositionSupplier.get();
    }
  }

  // Real-time tunable PID gains
  private final LoggedTunableNumber turretkP =
      new LoggedTunableNumber("Turret/kP", TURRET_GAINS.kP());
  private final LoggedTunableNumber turretkI =
      new LoggedTunableNumber("Turret/kI", TURRET_GAINS.kI());
  private final LoggedTunableNumber turretkD =
      new LoggedTunableNumber("Turret/kD", TURRET_GAINS.kD());
  private final LoggedTunableNumber turretkS =
      new LoggedTunableNumber("Turret/kS", TURRET_GAINS.kS());
  private final LoggedTunableNumber turretkV =
      new LoggedTunableNumber("Turret/kV", TURRET_GAINS.kV());
  private final LoggedTunableNumber turretkA =
      new LoggedTunableNumber("Turret/kA", TURRET_GAINS.kA());

  // Real-time tunable constraints
  private final LoggedTunableNumber turretMaxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity", TURRET_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber turretMaxAcceleration =
      new LoggedTunableNumber("Turret/MaxAcceleration", TURRET_CONSTRAINTS.maxAcceleration());

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  @AutoLogOutput(key = "Turret/State")
  @Getter
  @Setter
  private TurretState currentState = TurretState.STOW;

  @AutoLogOutput(key = "Turret/ManualOverride")
  @Getter
  @Setter
  private boolean manualOverride = false;

  @AutoLogOutput(key = "Turret/TargetPositionRots")
  private double targetPositionRots = 0.0;

  private Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.positionRots - targetPositionRots) < ANGLE_TOLERANCE_ROTS);

  /** Creates a new Turret subsystem. */
  public Turret(TurretIO io) {
    this.io = io;

    // Zero the turret on startup if available
    zeroTurretCRT();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (turretkP.hasChanged(hashCode())
        || turretkI.hasChanged(hashCode())
        || turretkD.hasChanged(hashCode())
        || turretkS.hasChanged(hashCode())
        || turretkV.hasChanged(hashCode())
        || turretkA.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              turretkP.get(),
              turretkI.get(),
              turretkD.get(),
              turretkS.get(),
              turretkV.get(),
              0.0,
              turretkA.get()));
    }

    if (turretMaxVelocity.hasChanged(hashCode()) || turretMaxAcceleration.hasChanged(hashCode())) {
      io.setConstraints(new Constraints(turretMaxVelocity.get(), turretMaxAcceleration.get()));
    }

    LoggedTracer.record("Turret");
  }

  public Command setStatePosition() {
    return run(() -> {
          if (!manualOverride) {
            targetPositionRots =
                MathUtil.clamp(
                    currentState.getTargetPosition().in(Rotations), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
            io.setPosition(targetPositionRots);
          }
        })
        .withName("Turret.SetStatePosition");
  }

  /**
   * Sets the turret to a position supplider
   *
   * @param positionRots target supplier position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(DoubleSupplier positionRots) {
    return run(() -> {
          setManualOverride(true);
          targetPositionRots =
              MathUtil.clamp(positionRots.getAsDouble(), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
          io.setPosition(positionRots.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
        .withName("Turret.SetPosition");
  }

  /**
   * Sets the turret to a position
   *
   * @param positionRots target position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(double positionRots) {
    return setPosition(() -> positionRots);
  }

  /**
   * Runs the turret at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the turret at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return run(() -> {
          setManualOverride(true);
          io.setVoltage(voltage.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
        .withName("Turret.SetVoltage");
  }

  /**
   * Runs the turret at a given voltage
   *
   * @param voltage target voltage
   * @return A command that repeatedly runs the turret at a voltage
   */
  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  /**
   * Manual control command for the turret. Bypasses state-based control.
   *
   * @param voltageSupplier Voltage supplier for manual control
   * @return A command for manual turret control
   */
  public Command manualControl(DoubleSupplier voltageSupplier) {
    return run(() -> {
          setManualOverride(true);
          io.setVoltage(voltageSupplier.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
        .withName("Turret.ManualControl");
  }

  /**
   * Gets the current position of the turret.
   *
   * @return The current turret angle
   */
  public Angle getPosition() {
    return Rotations.of(inputs.positionRots);
  }

  /**
   * Checks if the turret is at the target position.
   *
   * @return A trigger that is true if at target within tolerance
   */
  @AutoLogOutput(key = "Turret/AtSetpoint")
  public Trigger atSetpoint() {
    return atSetpoint;
  }

  /**
   * Sets the turret to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  /** Zeros the turret using two CANcoder sensors and the Chinese Remainder Theorem. */
  public void zeroTurretCRT() {
    // TODO: Implement CRT zeroing
  }
}
