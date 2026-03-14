package org.teamtitanium.subsystems.intake.rack;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.*;

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

public class IntakeRack extends SubsystemBase {
  private final LoggedTunableNumber rackkP =
      new LoggedTunableNumber("Intake/Rack/kP", RACK_GAINS.kP());
  private final LoggedTunableNumber rackkI =
      new LoggedTunableNumber("Intake/Rack/kI", RACK_GAINS.kI());
  private final LoggedTunableNumber rackkD =
      new LoggedTunableNumber("Intake/Rack/kD", RACK_GAINS.kD());
  private final LoggedTunableNumber rackkS =
      new LoggedTunableNumber("Intake/Rack/kS", RACK_GAINS.kS());
  private final LoggedTunableNumber rackkV =
      new LoggedTunableNumber("Intake/Rack/kV", RACK_GAINS.kV());
  private final LoggedTunableNumber rackkG =
      new LoggedTunableNumber("Intake/Rack/kG", RACK_GAINS.kG());
  private final LoggedTunableNumber rackkA =
      new LoggedTunableNumber("Intake/Rack/kA", RACK_GAINS.kA());

  private final LoggedTunableNumber rackMaxVelocity =
      new LoggedTunableNumber("Intake/Rack/MaxVelocity", RACK_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber rackMaxAcceleration =
      new LoggedTunableNumber("Intake/Rack/MaxAcceleration", RACK_CONSTRAINTS.maxAcceleration());

  public static final LoggedTunableNumber configRackNumber1 =
      new LoggedTunableNumber("Intake/Rack/ConfigNumber1", 0.0);
  public static final LoggedTunableNumber configRackNumber2 =
      new LoggedTunableNumber("Intake/Rack/ConfigNumber2", 0.0);

  private final IntakeRackIO io;
  private final IntakeRackIOInputsAutoLogged inputs = new IntakeRackIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Rack/TargetExtensionMeters")
  private double targetExtensionMeters = STOW_EXTENSION.in(Meters);

  private final Alert motorDisconnectedAlert =
      new Alert("Intake Rack Motor Disconnected", Alert.AlertType.kWarning);
  private final Debouncer disconnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  @AutoLogOutput(key = "Intake/Rack/AtSetpoint")
  private final Trigger atSetpoint =
      new Trigger(
          () ->
              Math.abs(inputs.positionMeters - targetExtensionMeters)
                  < EXTENSION_TOLERANCE.in(Meters));

  private final Debouncer stowCurrentDebouncer =
      new Debouncer(STOW_CURRENT_DEBOUNCE_TIME_SECS, DebounceType.kRising);
  private final Trigger stowCurrentTrigger =
      new Trigger(
          () ->
              stowCurrentDebouncer.calculate(
                  Math.abs(inputs.torqueCurrentAmps) >= STOW_STALL_CURRENT_THRESHOLD));

  private final Debouncer homeCurrentDebouncer =
      new Debouncer(HOMING_DEBOUNCE_TIME_SECS, DebounceType.kRising);
  private final Debouncer homeVelocityDebouncer =
      new Debouncer(HOMING_DEBOUNCE_TIME_SECS, DebounceType.kRising);
  private final Trigger homeCurrentTrigger =
      new Trigger(
          () ->
              homeCurrentDebouncer.calculate(
                  Math.abs(inputs.torqueCurrentAmps) >= HOMING_CURRENT_THRESHOLD_AMPS));
  private final Trigger homeVelocityTrigger =
      new Trigger(
          () ->
              homeVelocityDebouncer.calculate(
                  Math.abs(inputs.velocityRps) <= HOMING_VELOCITY_THRESHOLD_RPS));
  private boolean rackZeroed = false;

  public IntakeRack(IntakeRackIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Rack", inputs);

    motorDisconnectedAlert.set(disconnectedDebouncer.calculate(inputs.motorConnected));

    if (rackkP.hasChanged(hashCode())
        || rackkI.hasChanged(hashCode())
        || rackkD.hasChanged(hashCode())
        || rackkS.hasChanged(hashCode())
        || rackkV.hasChanged(hashCode())
        || rackkG.hasChanged(hashCode())
        || rackkA.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              rackkP.get(),
              rackkI.get(),
              rackkD.get(),
              rackkS.get(),
              rackkV.get(),
              rackkG.get(),
              rackkA.get()));
    }

    if (rackMaxVelocity.hasChanged(hashCode()) || rackMaxAcceleration.hasChanged(hashCode())) {
      io.setConstraints(new Constraints(rackMaxVelocity.get(), rackMaxAcceleration.get()));
    }

    LoggedTracer.record("Intake/Rack");
  }

  public Command setPosition(Supplier<Double> positionRots) {
    return runEnd(() -> io.setPosition(positionRots.get()), () -> io.setVoltage(0.0));
  }

  public Command setExtension(Supplier<Distance> positionSupplier) {
    return setExtension(() -> positionSupplier.get(), RACK_CONSTRAINTS);
  }

  public Command setExtension(Supplier<Distance> positionSupplier, Constraints constraints) {
    return run(() -> {
          double requestedMeters =
              MathUtil.clamp(
                  positionSupplier.get().in(Meters),
                  MIN_EXTENSION.in(Meters),
                  MAX_EXTENSION.in(Meters));
          targetExtensionMeters = requestedMeters;
          io.setPosition(metersToMotorRotations(requestedMeters), constraints);
        })
        .withName("IntakeRack.SetExtension");
  }

  public Command setExtension(Distance position) {
    return setExtension(() -> position);
  }

  public Command setExtensionEnd(Distance position) {
    return setExtension(position).until(atSetpoint(position));
  }

  public Command setVoltage(DoubleSupplier voltageSupplier) {
    return runEnd(() -> io.setVoltage(voltageSupplier.getAsDouble()), () -> io.stop())
        .withName("IntakeRack.SetVoltage");
  }

  public Command setVoltage(double volts) {
    return setVoltage(() -> volts);
  }

  public Command stow() {
    return Commands.repeatingSequence(
            setExtension(STOW_EXTENSION).until(stowCurrentTrigger),
            setExtensionEnd(getExtension().plus(Inches.of(2.0))).withTimeout(2.0))
        .until(atSetpoint(STOW_EXTENSION));
  }

  public Command stop() {
    return setVoltage(0.0).withName("IntakeRack.Stop");
  }

  public Distance getExtension() {
    return Meters.of(inputs.positionMeters);
  }

  public Trigger atSetpoint() {
    return atSetpoint;
  }

  private Trigger atSetpoint(Distance targetDistance) {
    return new Trigger(
        () ->
            Math.abs(inputs.positionMeters - targetDistance.in(Meters))
                < EXTENSION_TOLERANCE.in(Meters));
  }

  // public Trigger stowStallCurrent() {
  //   return stowCurrentTrigger;
  // }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public Command zeroIntake() {
    return Commands.sequence(
        setVoltage(HOMING_VOLTAGE_VOLTS).until(homeCurrentTrigger.and(homeVelocityTrigger)),
        Commands.runOnce(
            () -> {
              io.setMotorPosition(0.0);
              rackZeroed = true;
            }));
  }
}
