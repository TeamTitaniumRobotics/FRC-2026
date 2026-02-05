package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
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

  private final LoggedTunableNumber configNumber =
      new LoggedTunableNumber("Turret/AdjustableNumber", 0.0);

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

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

    if (configNumber.hasChanged(hashCode())) {
      Logger.recordOutput(
          "Turret/TargetConfigAngle",
          getTargetAngle(Degrees.of(configNumber.get()), getPosition()));
    }

    // setDefaultCommand(setPosition(() -> Degrees.of(configNumber.get())));

    LoggedTracer.record("Turret");
  }

  /**
   * Sets the turret to the stowed position
   *
   * @return A command that sets the turret to the stowed position
   */
  public Command stow() {
    return setPosition(TURRET_STOW_ANGLE);
  }

  /**
   * Tracks the turret to the robot state turret setpoint
   *
   * @return A command that repeatedly sets the turret to the robot state turret setpoint
   */
  public Command track() {
    return setPosition(() -> RobotState.getInstance().getTurretSetpoint());
  }

  public Angle getTargetAngle(Angle targetAngle, Angle currentAngle) {
    double deltaAngleRad = targetAngle.minus(currentAngle).in(Radians);
    if (deltaAngleRad > Math.PI) {
      deltaAngleRad -= 2 * Math.PI;
    } else if (deltaAngleRad < -Math.PI) {
      deltaAngleRad += 2 * Math.PI;
    }

    Logger.recordOutput("Turret/DeltaAngle", deltaAngleRad);

    double optimalAngleRad = currentAngle.plus(Radians.of(deltaAngleRad)).in(Radians);
    Logger.recordOutput("Turret/OptimalAngle", optimalAngleRad);
    if (currentAngle.plus(Radians.of(deltaAngleRad)).in(Radians) % (2 * Math.PI)
        == currentAngle.minus(Radians.of(deltaAngleRad)).in(Radians) % (2 * Math.PI)) {
      // If both directions are equally optimal, prefer the one closer to zero
      if (optimalAngleRad > 0) {
        optimalAngleRad = currentAngle.minus(Radians.of(Math.abs(deltaAngleRad))).in(Radians);
      } else {
        optimalAngleRad = currentAngle.plus(Radians.of(Math.abs(deltaAngleRad))).in(Radians);
      }
    }
    if (optimalAngleRad > Rotations.of(MAX_ANGLE_ROTS).in(Radians)) {
      optimalAngleRad -= 2 * Math.PI;
    } else if (optimalAngleRad < Rotations.of(MIN_ANGLE_ROTS).in(Radians)) {
      optimalAngleRad += 2 * Math.PI;
    }
    return Radians.of(optimalAngleRad);
  }

  /**
   * Sets the turret to a position supplider
   *
   * @param positionRots target supplier position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(Supplier<Angle> position) {
    return run(() -> {
          // targetPositionRots =
          //     MathUtil.clamp(position.get().in(Rotations), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
          // io.setPosition(position.get().in(Rotations));
          targetPositionRots = getTargetAngle(position.get(), getPosition()).in(Rotations);
          io.setPosition(targetPositionRots);
        })
        .withName("Turret.SetPosition");
  }

  /**
   * Sets the turret to a position
   *
   * @param position target position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(Angle position) {
    return setPosition(() -> position);
  }

  /**
   * Runs the turret at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the turret at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return runEnd(() -> io.setVoltage(voltage.getAsDouble()), () -> io.setVoltage(0.0))
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
