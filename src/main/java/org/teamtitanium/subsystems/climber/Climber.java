package org.teamtitanium.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class Climber extends SubsystemBase {
  private final LoggedTunableNumber climberkPSlot0 =
      new LoggedTunableNumber("Climber/Slot0/kP", CLIMBER_GAINS_SLOT_0.kP());
  private final LoggedTunableNumber climberkISlot0 =
      new LoggedTunableNumber("Climber/Slot0/kI", CLIMBER_GAINS_SLOT_0.kI());
  private final LoggedTunableNumber climberkDSlot0 =
      new LoggedTunableNumber("Climber/Slot0/kD", CLIMBER_GAINS_SLOT_0.kD());
  private final LoggedTunableNumber climberkSSlot0 =
      new LoggedTunableNumber("Climber/Slot0/kS", CLIMBER_GAINS_SLOT_0.kS());
  private final LoggedTunableNumber climberkVSlot0 =
      new LoggedTunableNumber("Climber/Slot0/kV", CLIMBER_GAINS_SLOT_0.kV());
  private final LoggedTunableNumber climberkGSlot0 =
      new LoggedTunableNumber("Climber/Slot0/kG", CLIMBER_GAINS_SLOT_0.kG());
  private final LoggedTunableNumber climberkASlot0 =
      new LoggedTunableNumber("Climber/Slot0/kA", CLIMBER_GAINS_SLOT_0.kA());

  private final LoggedTunableNumber climberkPSlot1 =
      new LoggedTunableNumber("Climber/Slot1/kP", CLIMBER_GAINS_SLOT_1.kP());
  private final LoggedTunableNumber climberkISlot1 =
      new LoggedTunableNumber("Climber/Slot1/kI", CLIMBER_GAINS_SLOT_1.kI());
  private final LoggedTunableNumber climberkDSlot1 =
      new LoggedTunableNumber("Climber/Slot1/kD", CLIMBER_GAINS_SLOT_1.kD());
  private final LoggedTunableNumber climberkSSlot1 =
      new LoggedTunableNumber("Climber/Slot1/kS", CLIMBER_GAINS_SLOT_1.kS());
  private final LoggedTunableNumber climberkVSlot1 =
      new LoggedTunableNumber("Climber/Slot1/kV", CLIMBER_GAINS_SLOT_1.kV());
  private final LoggedTunableNumber climberkGSlot1 =
      new LoggedTunableNumber("Climber/Slot1/kG", CLIMBER_GAINS_SLOT_1.kG());
  private final LoggedTunableNumber climberkASlot1 =
      new LoggedTunableNumber("Climber/Slot1/kA", CLIMBER_GAINS_SLOT_1.kA());

  private final LoggedTunableNumber climberMaxVelocity =
      new LoggedTunableNumber("Climber/MaxVelocity", CLIMBER_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber climberMaxAcceleration =
      new LoggedTunableNumber("Climber/MaxAcceleration", CLIMBER_CONSTRAINTS.maxAcceleration());

  public final LoggedTunableNumber configClimberNumber1 =
      new LoggedTunableNumber("Climber/ConfigNumber1", 0.0);
  public final LoggedTunableNumber configClimberNumber2 =
      new LoggedTunableNumber("Climber/ConfigNumber2", 0.0);

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  @AutoLogOutput(key = "Climber/TargetExtensionMeters")
  private double targetExtensionMeters = CLIMBER_STOW_EXTENSION.in(Meters);

  @AutoLogOutput(key = "Climber/IsHomed")
  private boolean isHomed = false;

  private final Alert motorDisconnectedAlert =
      new Alert("Climber Motor Disconnected", Alert.AlertType.kWarning);
  private final Debouncer disconnectedDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final Trigger atSetpoint =
      new Trigger(
          () ->
              Math.abs(inputs.positionMeters - targetExtensionMeters)
                  < CLIMBER_TOLERANCE.in(Meters));

  private final Debouncer homeCurrentDebouncer =
      new Debouncer(HOMING_DEBOUNCE_TIME_SECS, DebounceType.kRising);
  private final Debouncer homeVelocityDebouncer =
      new Debouncer(HOMING_DEBOUNCE_TIME_SECS, DebounceType.kRising);

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    motorDisconnectedAlert.set(disconnectedDebouncer.calculate(!inputs.motorConnected));

    if (climberkPSlot0.hasChanged(hashCode())
        || climberkISlot0.hasChanged(hashCode())
        || climberkDSlot0.hasChanged(hashCode())
        || climberkSSlot0.hasChanged(hashCode())
        || climberkVSlot0.hasChanged(hashCode())
        || climberkGSlot0.hasChanged(hashCode())
        || climberkASlot0.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              climberkPSlot0.get(),
              climberkISlot0.get(),
              climberkDSlot0.get(),
              climberkSSlot0.get(),
              climberkVSlot0.get(),
              climberkGSlot0.get(),
              climberkASlot0.get()),
          0);
    }

    if (climberkPSlot1.hasChanged(hashCode())
        || climberkISlot1.hasChanged(hashCode())
        || climberkDSlot1.hasChanged(hashCode())
        || climberkSSlot1.hasChanged(hashCode())
        || climberkVSlot1.hasChanged(hashCode())
        || climberkGSlot1.hasChanged(hashCode())
        || climberkASlot1.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              climberkPSlot1.get(),
              climberkISlot1.get(),
              climberkDSlot1.get(),
              climberkSSlot1.get(),
              climberkVSlot1.get(),
              climberkGSlot1.get(),
              climberkASlot1.get()),
          1);
    }

    if (climberMaxVelocity.hasChanged(hashCode())
        || climberMaxAcceleration.hasChanged(hashCode())) {
      io.setConstraints(new Constraints(climberMaxVelocity.get(), climberMaxAcceleration.get()));
    }

    LoggedTracer.record("Climber");
  }

  public Command setPosition(Supplier<Double> positionRots) {
    return runEnd(() -> io.setPosition(positionRots.get(), 0), () -> io.stop());
  }

  public Command setExtension(Supplier<Distance> positionSupplier) {
    return run(() -> {
          double requestedMeters =
              MathUtil.clamp(
                  positionSupplier.get().in(Meters),
                  CLIMBER_MIN_EXTENSION.in(Meters),
                  CLIMBER_MAX_EXTENSION.in(Meters));
          targetExtensionMeters = requestedMeters;
          io.setPosition(metersToMechRotations(requestedMeters), 0);
        })
        .withName("Climber.SetExtension");
  }

  public Command setExtension(Distance position) {
    return setExtension(() -> position);
  }

  public Command setVoltage(DoubleSupplier voltageSupplier) {
    return runEnd(() -> io.setVoltage(voltageSupplier.getAsDouble()), () -> io.stop())
        .withName("Climber.SetVoltage");
  }

  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  public Command stop() {
    return Commands.runOnce(() -> io.stop());
  }

  public double getExtensionMeters() {
    return inputs.positionMeters;
  }

  public Trigger atSetpoint() {
    return atSetpoint;
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public void zero() {
    io.setMotorPosition(0.0);
  }
}
