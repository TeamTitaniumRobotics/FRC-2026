package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class Turret extends SubsystemBase {
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

  private double targetPositionRots = 0.0;
  private Trigger atSetpoint =
      new Trigger(
          () -> Math.abs(inputs.positionRots - targetPositionRots) < POSITION_TOLERANCE_ROTS);

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

    Logger.recordOutput("Turret/TargetPosition", targetPositionRots);

    LoggedTracer.record("Turret");
  }

  /**
   * Sets the turret to a position supplider
   *
   * @param positionRots target supplier position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(DoubleSupplier positionRots) {
    return run(() -> {
          targetPositionRots =
              MathUtil.clamp(positionRots.getAsDouble(), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
          io.setPosition(positionRots.getAsDouble());
        })
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
    return run(() -> io.setVoltage(voltage.getAsDouble())).withName("Turret.SetVoltage");
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
