package org.teamtitanium.subsystems.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.teamtitanium.subsystems.genericroller.GenericRoller;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class Spindexer extends GenericRoller {
  public static final int SPINDEXER_MOTOR_ID = 35;
  public static final CANBus SPINDEXER_CAN_BUS = Constants.RIO_CAN_BUS;

  public static final boolean SPINDEXER_INVERTED = false;

  public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0.0);
  public static final AngularVelocity FEED_VELOCITY = RotationsPerSecond.of(24.0);

  public static final double STATOR_CURRENT_LIMIT = 40.0;
  public static final double SUPPLY_CURRENT_LIMIT = 30.0;

  public static final double SPINDEXER_GEAR_RATIO = 1.0;

  public static final Gains SPINDEXER_GAINS = new Gains(6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  public static final Constraints SPINDEXER_CONSTRAINTS = new Constraints(24.0, 36.0);

  public static final DCMotor SPINDEXER_MOTOR_GEARBOX = DCMotor.getKrakenX44(1);
  public static final double SPINDEXER_MOI = 0.08;

  public static final GenericRollerConstants CONSTANTS =
      new GenericRollerConstants(
          SPINDEXER_MOTOR_ID,
          SPINDEXER_CAN_BUS,
          SPINDEXER_GEAR_RATIO,
          SPINDEXER_GAINS,
          SPINDEXER_CONSTRAINTS,
          STATOR_CURRENT_LIMIT,
          SUPPLY_CURRENT_LIMIT,
          SPINDEXER_INVERTED,
          true);

  public Spindexer(GenericRollerIO io) {
    super("Spindexer", io);
  }

  @Override
  public Command idle() {
    return setVelocity(IDLE_VELOCITY);
  }

  public Command feed() {
    return setVelocity(FEED_VELOCITY);
  }
}
