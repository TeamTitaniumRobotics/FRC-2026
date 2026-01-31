package org.teamtitanium.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.hood.HoodConstants.*;

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

public class Hood extends SubsystemBase {
  public enum HoodState {
    STOWED(() -> HOOD_STOW_ANGLE_ROTS),
    SHOOT(() -> RobotState.getInstance().getHoodSetpoint()),
    PASS(() -> RobotState.getInstance().getHoodSetpoint()),
    EJECT(() -> HOOD_EJECT_ANGLE_ROTS);

    private final Supplier<Angle> targetPositionSupplier;

    private HoodState(Supplier<Angle> targetPositionSupplier) {
      this.targetPositionSupplier = targetPositionSupplier;
    }

    public Angle getTargetPosition() {
      return targetPositionSupplier.get();
    }
  }

  // Real-time tunable PID gains
  private final LoggedTunableNumber hoodkP = new LoggedTunableNumber("Hood/kP", HOOD_GAINS.kP());
  private final LoggedTunableNumber hoodkI = new LoggedTunableNumber("Hood/kI", HOOD_GAINS.kI());
  private final LoggedTunableNumber hoodkD = new LoggedTunableNumber("Hood/kD", HOOD_GAINS.kD());
  private final LoggedTunableNumber hoodkS = new LoggedTunableNumber("Hood/kS", HOOD_GAINS.kS());
  private final LoggedTunableNumber hoodkV = new LoggedTunableNumber("Hood/kV", HOOD_GAINS.kV());
  private final LoggedTunableNumber hoodkG = new LoggedTunableNumber("Hood/kG", HOOD_GAINS.kG());
  private final LoggedTunableNumber hoodkA = new LoggedTunableNumber("Hood/kA", HOOD_GAINS.kA());

  // Real-time tunable constraints
  private final LoggedTunableNumber hoodMaxVelocity =
      new LoggedTunableNumber("Hood/MaxVelocity", HOOD_MOTION_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber hoodMaxAcceleration =
      new LoggedTunableNumber("Hood/MaxAcceleration", HOOD_MOTION_CONSTRAINTS.maxAcceleration());

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  @AutoLogOutput(key = "Hood/State")
  @Getter
  @Setter
  private HoodState currentState = HoodState.STOWED;

  @AutoLogOutput(key = "Hood/ManualOverride")
  @Getter
  @Setter
  private boolean manualOverride = false;

  @AutoLogOutput(key = "Hood/TargetPositionRots")
  private double targetPositionRots = 0.0;

  private Trigger atSetpoint =
      new Trigger(
          () -> Math.abs(inputs.positionRots - targetPositionRots) < POSITION_TOLERANCE_ROTS);

  /** Creates a new Hood subsystem. */
  public Hood(HoodIO io) {
    this.io = io;
    setDefaultCommand(setStatePosition());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

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

    LoggedTracer.record("Hood");
  }

  /**
   * Sets the hood to the current state's target position
   *
   * @return A command that repeatedly sets the hood to the current state's target position
   */
  private Command setStatePosition() {
    return run(() -> {
          if (!manualOverride) {
            targetPositionRots =
                MathUtil.clamp(
                    currentState.getTargetPosition().in(Rotations), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
            io.setPosition(targetPositionRots);
          }
        })
        .withName("Hood.SetStatePosition");
  }

  /**
   * Sets the hood to a position supplier
   *
   * @param positionRots target supplier position for the hood
   * @return A command that repeatedly sets the hood to a position
   */
  public Command setPosition(DoubleSupplier positionRots) {
    return run(() -> {
          setManualOverride(true);
          targetPositionRots =
              MathUtil.clamp(positionRots.getAsDouble(), MIN_ANGLE_ROTS, MAX_ANGLE_ROTS);
          io.setPosition(positionRots.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
        .withName("Hood.SetPosition");
  }

  /**
   * Sets the hood to a position
   *
   * @param positionRots target position for the hood
   * @return A command that repeatedly sets the hood to a position
   */
  public Command setPosition(double positionRots) {
    return setPosition(() -> positionRots);
  }

  /**
   * Runs the hood at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the hood at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return run(() -> {
          setManualOverride(true);
          io.setVoltage(voltage.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
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
    return setVoltage(0.0); // TODO: Implement current & velocity zeroing
  }

  /**
   * Manual control command for the hood. Bypasses state-based control.
   *
   * @param voltageSupplier Voltage supplier for manual control
   * @return A command for manual hood control
   */
  public Command manualControl(DoubleSupplier voltageSupplier) {
    return run(() -> {
          setManualOverride(true);
          io.setVoltage(voltageSupplier.getAsDouble());
        })
        .finallyDo(() -> setManualOverride(false))
        .withName("Hood.ManualControl");
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
  @AutoLogOutput(key = "Hood/AtSetpoint")
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
