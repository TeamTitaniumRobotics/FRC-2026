package org.teamtitanium.subsystems.genericroller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class GenericRoller extends SubsystemBase {
  private final LoggedTunableNumber rollerkP;
  private final LoggedTunableNumber rollerkI;
  private final LoggedTunableNumber rollerkD;
  private final LoggedTunableNumber rollerkS;
  private final LoggedTunableNumber rollerkV;
  private final LoggedTunableNumber rollerkA;

  private final LoggedTunableNumber rollerMaxVelocity;
  private final LoggedTunableNumber rollerMaxAcceleration;

  public final LoggedTunableNumber configurableNumber;

  private final String name;

  private final GenericRollerIO io;
  protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  public GenericRoller(String name, GenericRollerIO io) {
    this.name = name;
    this.io = io;

    rollerkP = new LoggedTunableNumber(name + "/Roller/kP", 0.0);
    rollerkI = new LoggedTunableNumber(name + "/Roller/kI", 0.0);
    rollerkD = new LoggedTunableNumber(name + "/Roller/kD", 0.0);
    rollerkS = new LoggedTunableNumber(name + "/Roller/kS", 0.0);
    rollerkV = new LoggedTunableNumber(name + "/Roller/kV", 0.0);
    rollerkA = new LoggedTunableNumber(name + "/Roller/kA", 0.0);

    rollerMaxVelocity = new LoggedTunableNumber(name + "/Roller/Max Velocity", 0.0);
    rollerMaxAcceleration = new LoggedTunableNumber(name + "/Roller/Max Acceleration", 0.0);

    configurableNumber = new LoggedTunableNumber(name + "/Roller/ConfigurableNumber", 0.0);

    motorDisconnected = new Alert(name + " Roller Motor Disconnected", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name + "/Roller", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));

    if (rollerkP.hasChanged(hashCode())
        || rollerkI.hasChanged(hashCode())
        || rollerkD.hasChanged(hashCode())
        || rollerkS.hasChanged(hashCode())
        || rollerkV.hasChanged(hashCode())
        || rollerkA.hasChanged(hashCode())) {
      setGains(
          new Gains(
              rollerkP.get(),
              rollerkI.get(),
              rollerkD.get(),
              rollerkS.get(),
              rollerkV.get(),
              0.0,
              rollerkA.get()));
    }

    if (rollerMaxVelocity.hasChanged(hashCode()) || rollerMaxAcceleration.hasChanged(hashCode())) {
      setConstraints(new Constraints(rollerMaxVelocity.get(), rollerMaxAcceleration.get()));
    }

    LoggedTracer.record(name + "/Roller");
  }

  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return run(() -> io.setVelocity(velocitySupplier.get().in(RotationsPerSecond)));
  }

  public Command setVelocity(AngularVelocity velocity) {
    return setVelocity(() -> velocity);
  }

  /**
   * Directly applies a velocity to the roller IO without creating a command. Intended for use
   * inside an already-running command (e.g. applySubStates).
   */
  protected void applyVelocity(AngularVelocity velocity) {
    io.setVelocity(velocity.in(RotationsPerSecond));
  }

  public Command setVoltage(Supplier<Double> voltageSupplier) {
    return run(() -> io.setVoltage(voltageSupplier.get()));
  }

  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  public Command stop() {
    return run(io::stop);
  }

  public double getVelocity() {
    return inputs.velocityRps;
  }

  public void setGains(Gains gains) {
    io.setGains(gains);
  }

  public void setConstraints(Constraints constraints) {
    io.setConstraints(constraints);
  }

  public record GenericRollerConstants(
      int motorId,
      CANBus canbus,
      double reduction,
      Gains gains,
      Constraints constraints,
      double statorCurrentLimit,
      double supplyCurrentLimit,
      boolean inverted,
      boolean brake) {}
}
