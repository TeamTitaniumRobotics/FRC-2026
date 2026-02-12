package org.teamtitanium.subsystems.intake.roller;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamtitanium.subsystems.genericroller.GenericRoller;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class IntakeRoller extends GenericRoller {
  public static final GenericRollerConstants constants =
      new GenericRollerConstants(
          20,
          CANBus.roboRIO(),
          1.0,
          new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
          new Constraints(0.0, 0.0),
          60.0,
          40.0,
          false,
          true);

  public IntakeRoller(GenericRollerIO io) {
    super("Intake", io);
  }

  public Command intake() {
    return setVoltage(3.0);
  }
}
