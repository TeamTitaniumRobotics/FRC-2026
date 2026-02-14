package org.teamtitanium.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants;
import org.teamtitanium.subsystems.intake.IntakeConstants.RollerConstants;
import org.teamtitanium.subsystems.intake.rack.IntakeRack;
import org.teamtitanium.subsystems.intake.roller.IntakeRoller;

public class Intake {
  private final IntakeRack rack;
  private final IntakeRoller roller;

  public Intake(IntakeRack rack, IntakeRoller roller) {
    this.rack = rack;
    this.roller = roller;
  }

  public Command stow() {
    return Commands.parallel(
        rack.setExtension(RackConstants.STOW_EXTENSION),
        roller.setVelocity(RollerConstants.IDLE_VELOCITY));
  }

  public Command intake() {
    return Commands.parallel(
        rack.setExtension(RackConstants.DEPLOY_EXTENSION), roller.setVoltage(6.0));
  }
}
