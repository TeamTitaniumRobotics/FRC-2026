package org.teamtitanium.subsystems.genericroller;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;

public class GenericRoller extends SubsystemBase {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber maxVelocity;
  private final LoggedTunableNumber maxAcceleration;

  private final String name;

  private final GenericRollerIO io;
  protected final GenericRollerIOInputsAutoLogged inputs = new GenericRollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  public GenericRoller(String name, GenericRollerIO io) {
    this.name = name;
    this.io = io;

    kP = new LoggedTunableNumber(name + "/Roller/kP", 0.0);
    kI = new LoggedTunableNumber(name + "/Roller/kI", 0.0);
    kD = new LoggedTunableNumber(name + "/Roller/kD", 0.0);
    kS = new LoggedTunableNumber(name + "/Roller/kS", 0.0);
    kV = new LoggedTunableNumber(name + "/Roller/kV", 0.0);
    kA = new LoggedTunableNumber(name + "/Roller/kA", 0.0);

    maxVelocity = new LoggedTunableNumber(name + "/Roller/Max Velocity", 0.0);
    maxAcceleration = new LoggedTunableNumber(name + "/Roller/Max Acceleration", 0.0);

    motorDisconnected = new Alert(name + " Roller Motor Disconnected", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name + "/Roller", inputs);

    motorDisconnected.set(!motorConnectedDebouncer.calculate(inputs.motorConnected));

    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())) {
      setGains(new Gains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), 0.0, kA.get()));
    }

    if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
      setConstraints(new Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    LoggedTracer.record(name + "/Roller");
  }

  public Command setVelocity(double velocityRps) {
    return run(() -> io.setVelocity(velocityRps));
  }

  public Command setVoltage(double voltage) {
    return run(() -> io.setVoltage(voltage));
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
