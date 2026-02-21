package org.teamtitanium.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants;
import org.teamtitanium.subsystems.intake.IntakeConstants.RollerConstants;
import org.teamtitanium.subsystems.intake.rack.IntakeRack;
import org.teamtitanium.subsystems.intake.roller.IntakeRoller;

public class Intake {
  /** Independent states for the intake subsystem group. */
  public enum IntakeState {
    STOW,
    INTAKE,
    AGITATE,
    EJECT
  }

  private final IntakeRack rack;
  private final IntakeRoller roller;

  @Getter
  @Setter
  @AutoLogOutput(key = "Intake/State")
  private IntakeState state = IntakeState.STOW;

  public Intake(IntakeRack rack, IntakeRoller roller) {
    this.rack = rack;
    this.roller = roller;
  }

  /**
   * Returns a command that continuously resolves the current {@link IntakeState} into the
   * appropriate rack and roller commands. This should be run as a long-lived command (e.g. via
   * whileTrue on a Trigger that is always active during teleop/auto).
   */
  public Command applySubStates() {
    return Commands.run(
            () -> {
              Logger.recordOutput("Intake/State", state.name());
            })
        .alongWith(
            Commands.select(
                    java.util.Map.of(
                        IntakeState.STOW, stow(),
                        IntakeState.INTAKE, intake(),
                        IntakeState.AGITATE, agitate(),
                        IntakeState.EJECT, eject()),
                    () -> state)
                .repeatedly())
        .ignoringDisable(true)
        .withName("Intake.ApplySubStates");
  }

  // ---- Low-level command factories (package-visible for direct use in autos) ----

  public Command stow() {
    return Commands.parallel(
        rack.setExtension(RackConstants.STOW_EXTENSION),
        roller.setVelocity(RollerConstants.IDLE_VELOCITY));
  }

  public Command intake() {
    return Commands.parallel(
        rack.setExtension(RackConstants.DEPLOY_EXTENSION),
        roller.setVelocity(RollerConstants.INTAKE_VELOCITY));
  }

  public Command agitate() {
    return Commands.parallel(
        roller.setVelocity(RollerConstants.INTAKE_VELOCITY),
        Commands.repeatingSequence(
            rack.setExtension(RackConstants.AGITATE_EXTENSION).until(rack.atSetpoint()),
            rack.setExtension(RackConstants.STOW_EXTENSION).until(rack.atSetpoint())));
  }

  public Command eject() {
    return Commands.parallel(
        rack.setExtension(RackConstants.DEPLOY_EXTENSION),
        roller.setVelocity(RollerConstants.INTAKE_VELOCITY.times(-1.0)));
  }
}
