package org.teamtitanium.subsystems.intake.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RollerConstants.*;

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

public class IntakeRoller extends SubsystemBase {
  private final LoggedTunableNumber rollerkP =
      new LoggedTunableNumber("Intake/Roller/kP", ROLLER_GAINS.kP());
  private final LoggedTunableNumber rollerkI =
      new LoggedTunableNumber("Intake/Roller/kI", ROLLER_GAINS.kI());
  private final LoggedTunableNumber rollerkD =
      new LoggedTunableNumber("Intake/Roller/kD", ROLLER_GAINS.kD());
  private final LoggedTunableNumber rollerkS =
      new LoggedTunableNumber("Intake/Roller/kS", ROLLER_GAINS.kS());
  private final LoggedTunableNumber rollerkV =
      new LoggedTunableNumber("Intake/Roller/kV", ROLLER_GAINS.kV());
  private final LoggedTunableNumber rollerkA =
      new LoggedTunableNumber("Intake/Roller/kA", ROLLER_GAINS.kA());

  private final LoggedTunableNumber rollerMaxVelocity =
      new LoggedTunableNumber("Intake/Roller/Max Velocity", ROLLER_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber rollerMaxAcceleration =
      new LoggedTunableNumber(
          "Intake/Roller/Max Acceleration", ROLLER_CONSTRAINTS.maxAcceleration());

  private final LoggedTunableNumber configurableNumber =
      new LoggedTunableNumber("Intake/Roller/ConfigurableNumber", 0.0);

  private final IntakeRollerIO io;
  private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnected;

  public IntakeRoller(IntakeRollerIO io) {
    this.io = io;

    motorDisconnected = new Alert("Intake Roller Motor Disconnected", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Roller", inputs);

    motorDisconnected.set(
        !motorConnectedDebouncer.calculate(
            inputs.leftMotorConnected && inputs.rightMotorConnected));

    if (rollerkP.hasChanged(hashCode())
        || rollerkI.hasChanged(hashCode())
        || rollerkD.hasChanged(hashCode())
        || rollerkS.hasChanged(hashCode())
        || rollerkV.hasChanged(hashCode())
        || rollerkA.hasChanged(hashCode())) {
      io.setGains(
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
      io.setConstraints(new Constraints(rollerMaxVelocity.get(), rollerMaxAcceleration.get(), 0.0));
    }

    LoggedTracer.record("Intake/Roller");
  }

  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return run(() -> io.setVelocity(velocitySupplier.get().in(RotationsPerSecond)));
  }

  public Command setVelocity(AngularVelocity velocity) {
    return setVelocity(() -> velocity);
  }

  public Command setVoltage(Supplier<Double> voltageSupplier) {
    return run(() -> io.setVoltage(voltageSupplier.get()));
  }

  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  public Command stop() {
    return run(() -> io.stop());
  }
}
