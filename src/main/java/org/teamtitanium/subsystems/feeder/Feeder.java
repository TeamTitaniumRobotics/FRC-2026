package org.teamtitanium.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.teamtitanium.subsystems.genericroller.GenericRoller;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.TunerConstants;

public class Feeder extends GenericRoller {
  public static final int FEEDER_MOTOR_ID = 40;
  public static final CANBus FEEDER_CAN_BUS = TunerConstants.kCANBus;

  public static final boolean FEEDER_INVERTED = true;

  public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0.0);
  public static final AngularVelocity FEED_VELOCITY = RotationsPerSecond.of(24.0);

  public static final double STATOR_CURRENT_LIMIT = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT = 30.0;

  public static final double FEEDER_GEAR_RATIO = 2.0;

  public static final Gains FEEDER_GAINS = new Gains(0.0, 0.0, 0.0, 0.19, 0.0, 0.0, 0.0);
  public static final Constraints FEEDER_CONSTRAINTS = new Constraints(24.0, 36.0);

  public static final DCMotor FEEDER_MOTOR_GEARBOX = DCMotor.getKrakenX44(1);
  public static final double FEEDER_MOI = 0.08;

  public static final GenericRollerConstants CONSTANTS =
      new GenericRollerConstants(
          FEEDER_MOTOR_ID,
          FEEDER_CAN_BUS,
          FEEDER_GEAR_RATIO,
          FEEDER_GAINS,
          FEEDER_CONSTRAINTS,
          STATOR_CURRENT_LIMIT,
          SUPPLY_CURRENT_LIMIT,
          FEEDER_INVERTED,
          true);

  public Feeder(GenericRollerIO io) {
    super("Feeder", io);
  }

  @Override
  public Command idle() {
    return setVelocity(IDLE_VELOCITY);
  }

  public Command feed() {
    return setVelocity(FEED_VELOCITY);
  }

  public Trigger hasFuel =
      new Trigger(() -> true); // TODO: Replace with actual logic to determine if feeder has fuel
}
