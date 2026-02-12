package org.teamtitanium.subsystems.genericroller;

import com.ctre.phoenix6.CANBus;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class GenericRoller {

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
