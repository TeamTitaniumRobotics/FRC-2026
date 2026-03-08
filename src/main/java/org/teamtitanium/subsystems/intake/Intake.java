package org.teamtitanium.subsystems.intake;

import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants;
import org.teamtitanium.subsystems.intake.IntakeConstants.RollerConstants;
import org.teamtitanium.subsystems.intake.rack.IntakeRack;
import org.teamtitanium.subsystems.intake.roller.IntakeRoller;

public class Intake {
  /** Independent states for the intake subsystem group. */
  @RequiredArgsConstructor
  public enum IntakeState {
    STOW(() -> RackConstants.STOW_EXTENSION, () -> RollerConstants.IDLE_VELOCITY),
    INTAKE(() -> RackConstants.DEPLOY_EXTENSION, () -> RollerConstants.INTAKE_VELOCITY),
    AGITATE(() -> RackConstants.STOW_EXTENSION, () -> RollerConstants.INTAKE_VELOCITY),
    EJECT(() -> RackConstants.DEPLOY_EXTENSION, () -> RollerConstants.EJECT_VELOCITY);

    @Getter private final Supplier<Distance> rackDistance;
    @Getter private final Supplier<AngularVelocity> intakeVelocity;
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

    rack.setDefaultCommand(
        Commands.either(
            rack.setExtension(() -> state.getRackDistance().get(), AGITATE_CONSTRAINTS),
            rack.setExtension(() -> state.getRackDistance().get()),
            () -> state == IntakeState.AGITATE));
    // rack.setDefaultCommand(rack.setExtension(() -> state.getRackDistance().get()));
    roller.setDefaultCommand(roller.setVelocity(() -> state.getIntakeVelocity().get()));
    // roller.setDefaultCommand(
    //     roller.setVelocity(() -> RotationsPerSecond.of(roller.configurableNumber.get())));
  }

  public Trigger atSetpoint() {
    return rack.atSetpoint();
  }
}
