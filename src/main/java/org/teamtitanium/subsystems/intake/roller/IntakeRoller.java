package org.teamtitanium.subsystems.intake.roller;

import org.teamtitanium.subsystems.genericroller.GenericRoller;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.subsystems.intake.IntakeConstants;

public class IntakeRoller extends GenericRoller {
  public IntakeRoller(GenericRollerIO io) {
    super("Intake", io, IntakeConstants.RollerConstants.CONSTANTS);
  }
}
