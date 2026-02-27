package org.teamtitanium.subsystems.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamtitanium.subsystems.genericroller.GenericRoller;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public class Spindexer extends GenericRoller {
  /** Independent states for the spindexer. */
  @RequiredArgsConstructor
  public enum SpindexerState {
    IDLE(() -> 0.0),
    AGITATE(() -> 9.0),
    FEED(() -> 9.0);

    @Getter private final Supplier<Double> spindexerVelocity;
  }

  public static final int SPINDEXER_MOTOR_ID = 35;
  public static final CANBus SPINDEXER_CAN_BUS = Constants.RIO_CAN_BUS;

  public static final boolean SPINDEXER_INVERTED = true;

  public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0.0);
  public static final AngularVelocity AGITATE_VELOCITY = RotationsPerSecond.of(12.0);
  public static final AngularVelocity FEED_VELOCITY = RotationsPerSecond.of(24.0);

  public static final double STATOR_CURRENT_LIMIT = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;

  public static final double SPINDEXER_GEAR_RATIO = (38.0 / 12.0) * (46.0 / 20.0) * (36.0 / 18.0);

  public static final Gains SPINDEXER_GAINS = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
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
    super("Spindexer", io, CONSTANTS);
    setDefaultCommand(setVoltage(() -> state.getSpindexerVelocity().get()));
  }

  public Trigger hasFuel =
      new Trigger(() -> true); // TODO: Replace with actual logic to determine if spindexer has fuel
}
