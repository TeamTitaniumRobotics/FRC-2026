package org.teamtitanium.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.Robot;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class Hood extends SubsystemBase {
  // Real-time tunable PID gains
  private final LoggedTunableNumber hoodkP =
      new LoggedTunableNumber("Shooter/Hood/kP", HOOD_GAINS.kP());
  private final LoggedTunableNumber hoodkI =
      new LoggedTunableNumber("Shooter/Hood/kI", HOOD_GAINS.kI());
  private final LoggedTunableNumber hoodkD =
      new LoggedTunableNumber("Shooter/Hood/kD", HOOD_GAINS.kD());
  private final LoggedTunableNumber hoodkS =
      new LoggedTunableNumber("Shooter/Hood/kS", HOOD_GAINS.kS());
  private final LoggedTunableNumber hoodkV =
      new LoggedTunableNumber("Shooter/Hood/kV", HOOD_GAINS.kV());
  private final LoggedTunableNumber hoodkG =
      new LoggedTunableNumber("Shooter/Hood/kG", HOOD_GAINS.kG());
  private final LoggedTunableNumber hoodkA =
      new LoggedTunableNumber("Shooter/Hood/kA", HOOD_GAINS.kA());

  // Real-time tunable constraints
  private final LoggedTunableNumber hoodMaxVelocity =
      new LoggedTunableNumber("Shooter/Hood/MaxVelocity", HOOD_MOTION_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber hoodMaxAcceleration =
      new LoggedTunableNumber(
          "Shooter/Hood/MaxAcceleration", HOOD_MOTION_CONSTRAINTS.maxAcceleration());

  public final LoggedTunableNumber hoodConfigNumber1 =
      new LoggedTunableNumber("Shooter/Hood/ConfigNumber1", 0.0);
  public final LoggedTunableNumber hoodConfigNumber2 =
      new LoggedTunableNumber("Shooter/Hood/ConfigNumber2", 0.0);

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/Hood/TargetAngleRots")
  private double targetPositionRots = 0.0;

  private Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.positionRots - targetPositionRots) < ANGLE_TOLERANCE_ROTS);

  @AutoLogOutput(key = "Shooter/Hood/HoodZeroed")
  private boolean hoodZeroed = false;

  private final Debouncer currentDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);
  private final Debouncer velocityDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kRising);
  private final Trigger currentTrigger =
      new Trigger(
          () ->
              currentDebouncer.calculate(
                      Math.abs(inputs.torqueCurrentAmps) >= HoodConstants.ZERO_CURRENT_LIMIT)
                  && velocityDebouncer.calculate(
                      Math.abs(inputs.velocityRps) <= HoodConstants.ZERO_VELOCITY_LIMIT));

  /** Creates a new Hood subsystem. */
  public Hood(HoodIO io) {
    this.io = io;

    // RobotModeTriggers.teleop()
    //     .or(RobotModeTriggers.autonomous())
    //     .and(() -> !hoodZeroed)
    //     .onTrue(zeroHood());

    Robot.getCoastOverrideTrigger()
        .onTrue(runOnce(() -> this.setBrakeMode(false)).ignoringDisable(true))
        .onFalse(runOnce(() -> this.setBrakeMode(true)).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);

    if (hoodkP.hasChanged(hashCode())
        || hoodkI.hasChanged(hashCode())
        || hoodkD.hasChanged(hashCode())
        || hoodkS.hasChanged(hashCode())
        || hoodkV.hasChanged(hashCode())
        || hoodkG.hasChanged(hashCode())
        || hoodkA.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              hoodkP.get(),
              hoodkI.get(),
              hoodkD.get(),
              hoodkS.get(),
              hoodkV.get(),
              hoodkG.get(),
              hoodkA.get()));
    }

    if (hoodMaxVelocity.hasChanged(hashCode()) || hoodMaxAcceleration.hasChanged(hashCode())) {
      io.setConstraints(new Constraints(hoodMaxVelocity.get(), hoodMaxAcceleration.get()));
    }

    LoggedTracer.record("Shooter/Hood");
  }

  /**
   * Sets the hood to a position supplier
   *
   * @param positionSupplier target supplier position for the hood
   * @return A command that repeatedly sets the hood to a position
   */
  public Command setPosition(Supplier<Angle> positionSupplier) {
    return run(() -> {
          targetPositionRots =
              MathUtil.clamp(positionSupplier.get().in(Rotations), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
          io.setPosition(positionSupplier.get().in(Rotations));
        })
        .withName("Hood.SetPosition");
  }

  /**
   * Sets the hood to a position
   *
   * @param position target position for the hood
   * @return A command that repeatedly sets the hood to a position
   */
  public Command setPosition(Angle position) {
    return setPosition(() -> position);
  }

  /**
   * Runs the hood at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the hood at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return runEnd(() -> io.setVoltage(voltage.getAsDouble()), () -> io.stopMotor())
        .withName("Hood.SetVoltage");
  }

  /**
   * Runs the hood at a given voltage
   *
   * @param voltage target voltage
   * @return A command that repeatedly runs the hood at a voltage
   */
  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  public Command zeroHood() {
    return Commands.sequence(
        setVoltage(-1.0).until(currentTrigger),
        Commands.runOnce(
            () -> {
              io.setMotorPosition(0.0);
              hoodZeroed = true;
            }));
  }

  public void zeroMotor() {
    io.setMotorPosition(0.0);
  }

  public Command stop() {
    return Commands.runOnce(() -> io.stopMotor());
  }

  /**
   * Gets the current position of the hood.
   *
   * @return The current hood angle
   */
  public Angle getPosition() {
    return Rotations.of(inputs.positionRots);
  }

  /**
   * Checks if the hood is at the target position.
   *
   * @return A trigger that is true if at target within tolerance
   */
  @AutoLogOutput(key = "Shooter/Hood/AtSetpoint")
  public Trigger atSetpoint() {
    return atSetpoint;
  }

  /**
   * Sets the hood to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }
}
