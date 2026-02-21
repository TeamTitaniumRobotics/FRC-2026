package org.teamtitanium.subsystems.intake.rack;

import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

  public final LoggedTunableNumber configRackNumber =
      new LoggedTunableNumber("Intake/Rack/ConfigNumber", 0.0);

  private final IntakeRackIO io;
  private final IntakeRackIOInputsAutoLogged inputs = new IntakeRackIOInputsAutoLogged();

  @AutoLogOutput(key = "Intake/Rack/TargetExtensionMeters")
  private double targetExtensionMeters = STOW_EXTENSION.in(Meters);

  @AutoLogOutput(key = "Intake/Rack/IsHomed")
  private boolean isHomed = false;

  private final Alert motorDisconnectedAlert =
      new Alert("Intake Rack Motor Disconnected", Alert.AlertType.kWarning);
  private final Debouncer disconnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Trigger atSetpoint =
      new Trigger(
          () ->
              Math.abs(inputs.positionMeters - targetExtensionMeters)
                  < EXTENSION_TOLERANCE.in(Meters));

  private final Debouncer homeCurrentDebouncer =
      new Debouncer(HOMING_DEBOUNCE_TIME_SECS, DebounceType.kRising);
  private final Debouncer homeVelocityDebouncer =
      new Debouncer(HOMING_DEBOUNCE_TIME_SECS, DebounceType.kRising);

  private boolean homingSatisfied = false;

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
    return run(() -> {
          double requestedMeters =
              MathUtil.clamp(
                  positionSupplier.get().in(Meters),
                  MIN_EXTENSION.in(Meters),
                  MAX_EXTENSION.in(Meters));
          targetExtensionMeters = requestedMeters;
          io.setPosition(metersToMotorRotations(requestedMeters));
        })
        .withName("IntakeRack.SetExtension");
  }

  public Command setExtension(Distance position) {
    return setExtension(() -> position);
  }

  public Command setVoltage(DoubleSupplier voltageSupplier) {
    return runEnd(() -> io.setVoltage(voltageSupplier.getAsDouble()), () -> io.setVoltage(0.0))
        .withName("IntakeRack.SetVoltage");
  }

  public Command setVoltage(double volts) {
    return setVoltage(() -> volts);
  }

  public Command stop() {
    return setVoltage(0.0).withName("IntakeRack.Stop");
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

  public Command home() {
    return new FunctionalCommand(
            () -> {
              homingSatisfied = false;
              isHomed = false;
            },
            () -> {
              io.setVoltage(HOMING_VOLTAGE_VOLTS);
              boolean currentReady =
                  homeCurrentDebouncer.calculate(
                      inputs.supplyCurrentAmps >= HOMING_CURRENT_THRESHOLD_AMPS);
              boolean velocityReady =
                  homeVelocityDebouncer.calculate(
                      Math.abs(inputs.velocityRps) <= HOMING_VELOCITY_THRESHOLD_RPS);
              homingSatisfied = currentReady && velocityReady;
            },
            interrupted -> io.setVoltage(0.0),
            () -> homingSatisfied,
            this)
        .andThen(
            runOnce(
                () -> {
                  io.setMotorPosition(0.0);
                  targetExtensionMeters = MIN_EXTENSION.in(Meters);
                  isHomed = true;
                }))
        .andThen(setExtension(MIN_EXTENSION))
        .withName("IntakeRack.Home");
  }
}
