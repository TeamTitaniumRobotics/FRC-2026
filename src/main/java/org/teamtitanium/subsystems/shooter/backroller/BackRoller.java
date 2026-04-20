package org.teamtitanium.subsystems.shooter.backroller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.teamtitanium.subsystems.shooter.backroller.BackRollerConstants.BACK_ROLLER_CONSTRAINTS;
import static org.teamtitanium.subsystems.shooter.backroller.BackRollerConstants.BACK_ROLLER_GAINS;
import static org.teamtitanium.subsystems.shooter.backroller.BackRollerConstants.VELOCITY_TOLERANCE_RPS;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
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

public class BackRoller extends SubsystemBase {
  private static final LoggedTunableNumber rollerkP =
      new LoggedTunableNumber("Shooter/BackRoller/kP", BACK_ROLLER_GAINS.kP());
  private static final LoggedTunableNumber rollerkI =
      new LoggedTunableNumber("Shooter/BackRoller/kI", BACK_ROLLER_GAINS.kI());
  private static final LoggedTunableNumber rollerkD =
      new LoggedTunableNumber("Shooter/BackRoller/kD", BACK_ROLLER_GAINS.kD());
  private static final LoggedTunableNumber rollerkS =
      new LoggedTunableNumber("Shooter/BackRoller/kS", BACK_ROLLER_GAINS.kS());
  private static final LoggedTunableNumber rollerkV =
      new LoggedTunableNumber("Shooter/BackRoller/kV", BACK_ROLLER_GAINS.kV());
  private static final LoggedTunableNumber rollerkA =
      new LoggedTunableNumber("Shooter/BackRoller/kA", BACK_ROLLER_GAINS.kA());

  private static final LoggedTunableNumber rollerMaxVelocity =
      new LoggedTunableNumber(
          "Shooter/BackRoller/Max Velocity", BACK_ROLLER_CONSTRAINTS.maxVelocity());
  private static final LoggedTunableNumber rollerMaxAcceleration =
      new LoggedTunableNumber(
          "Shooter/BackRoller/Max Acceleration", BACK_ROLLER_CONSTRAINTS.maxAcceleration());

  public static final LoggedTunableNumber configurableNumber =
      new LoggedTunableNumber("Shooter/BackRoller/ConfigurableNumber", 0.0);

  private final BackRollerIO io;
  private final BackRollerIOInputsAutoLogged inputs = new BackRollerIOInputsAutoLogged();

  @AutoLogOutput(key = "Shooter/BackRoller/TargetVelocityRps")
  private double targetVelocityRps = 0.0;

  private final Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.velocityRps - targetVelocityRps) < VELOCITY_TOLERANCE_RPS);

  public BackRoller(BackRollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/BackRoller", inputs);

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
      io.setConstraints(new Constraints(rollerMaxVelocity.get(), rollerMaxAcceleration.get()));
    }

    LoggedTracer.record("Shooter/BackRoller");
  }

  /**
   * Sets the back roller to a supplied velocity.
   *
   * @param velocitySupplier target velocity supplier for the back roller
   * @return A command that repeatedly sets the back roller to a velocity
   */
  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return run(() -> {
          targetVelocityRps = velocitySupplier.get().in(RotationsPerSecond);
          io.setVelocity(targetVelocityRps);
        })
        .withName("BackRoller.SetVelocity");
  }

  /**
   * Sets the back roller to a velocity
   *
   * @param velocityRps target velocity for the back roller
   * @return A command that repeatedly sets the back roller to a velocity
   */
  public Command setVelocity(AngularVelocity velocity) {
    return setVelocity(() -> velocity);
  }

  /**
   * Runs the back roller at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the back roller at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return run(() -> io.setVoltage(voltage.getAsDouble())).withName("BackRoller.SetVoltage");
  }

  /**
   * Runs the back roller at a given voltage
   *
   * @param voltage target voltage
   * @return A command that repeatedly runs the back roller at a voltage
   */
  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  /**
   * Checks if the flywheel is at the target velocity.
   *
   * @return A trigger that is true if at target within tolerance
   */
  @AutoLogOutput(key = "Shooter/BackRoller/AtSetpoint")
  public Trigger atSetpoint() {
    return atSetpoint;
  }
}
