package org.teamtitanium.subsystems.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.genericroller.GenericRoller;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class Spindexer extends GenericRoller {
  /** Independent states for the spindexer. */
  public enum SpindexerState {
    IDLE,
    AGITATE,
    FEED
  }

  public static final int SPINDEXER_MOTOR_ID = 35;
  public static final CANBus SPINDEXER_CAN_BUS = Constants.RIO_CAN_BUS;

  public static final boolean SPINDEXER_INVERTED = false;

  public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0.0);
  public static final AngularVelocity AGITATE_VELOCITY = RotationsPerSecond.of(12.0);
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

  @Getter
  @Setter
  @AutoLogOutput(key = "Spindexer/State")
  private SpindexerState state = SpindexerState.IDLE;

  public Spindexer(GenericRollerIO io) {
    super("Spindexer", io);
  }

  /**
   * Returns a command that continuously resolves the current {@link SpindexerState} into the
   * appropriate roller velocity. This should be run as a long-lived command.
   */
  public Command applySubStates() {
    return run(() -> {
          Logger.recordOutput("Spindexer/State", state.name());
          switch (state) {
            case IDLE -> applyVelocity(IDLE_VELOCITY);
            case AGITATE -> applyVelocity(AGITATE_VELOCITY);
            case FEED -> applyVelocity(FEED_VELOCITY);
          }
        })
        .ignoringDisable(true)
        .withName("Spindexer.ApplySubStates");
  }

  @Override
  public Command idle() {
    return setVelocity(IDLE_VELOCITY);
  }

  public Command agitate() {
    return setVelocity(AGITATE_VELOCITY);
  }

  public Command feed() {
    return setVelocity(FEED_VELOCITY);
  }

  public Trigger hasFuel =
      new Trigger(() -> true); // TODO: Replace with actual logic to determine if spindexer has fuel
}
