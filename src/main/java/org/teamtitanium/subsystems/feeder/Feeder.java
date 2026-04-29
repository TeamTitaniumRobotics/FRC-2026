package org.teamtitanium.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
import org.teamtitanium.utils.LoggedTunableNumber;

public class Feeder extends GenericRoller {
  /** Independent states for the feeder. */
  @RequiredArgsConstructor
  public enum FeederState {
    IDLE(() -> IDLE_VELOCITY),
    // UNJAM(() -> RotationsPerSecond.of(Feeder.configNumber.get()).times(-1.0)),
    // FEED(() -> RotationsPerSecond.of(Feeder.configNumber.get()));
    UNJAM(() -> FEED_VELOCITY.times(-0.5)),
    FEED(() -> FEED_VELOCITY);

    @Getter private final Supplier<AngularVelocity> feederVelocity;
  }

  public static final LoggedTunableNumber configNumber =
      new LoggedTunableNumber("Feeder/Roller/ConfigShotNum", 0.0);

  public static final AngularVelocity IDLE_VELOCITY = RotationsPerSecond.of(0.0);
  public static final AngularVelocity FEED_VELOCITY = RotationsPerSecond.of(100.8);

  public static final int FEEDER_MOTOR_ID = 40;
  public static final CANBus FEEDER_CAN_BUS =
      Constants.getMode() == Constants.Mode.REAL ? Constants.CANIVORE : Constants.RIO_CAN_BUS;

  public static final boolean FEEDER_INVERTED = true;

  public static final double STATOR_CURRENT_LIMIT = 60.0;
  public static final double SUPPLY_CURRENT_LIMIT = 40.0;

  public static final double FEEDER_GEAR_RATIO = 0.5;

  public static final Gains FEEDER_GAINS = new Gains(0.15, 0.0, 0.0, 0.325, 0.065, 0.0, 0.0);
  public static final Constraints FEEDER_CONSTRAINTS = new Constraints(100.0, 400.0);

  public static final DCMotor FEEDER_MOTOR_GEARBOX = DCMotor.getKrakenX44(1);
  public static final double FEEDER_MOI = 0.08;

  private final Debouncer currentDebouncer = new Debouncer(1.0, DebounceType.kRising);
  private final Debouncer velocityDebouncer = new Debouncer(1.0, DebounceType.kRising);
  private final Trigger stallTrigger =
      new Trigger(
          () ->
              currentDebouncer.calculate(
                      Math.abs(inputs.torqueCurrentAmps) >= STATOR_CURRENT_LIMIT - 10)
                  && velocityDebouncer.calculate(Math.abs(inputs.velocityRps) <= 2.0));

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
          false);

  @Getter
  @Setter
  @AutoLogOutput(key = "Feeder/State")
  private FeederState state = FeederState.IDLE;

  private FeederState lastState = state;

  public Feeder(GenericRollerIO io) {
    super("Feeder", io, CONSTANTS);
    // stallTrigger.onTrue(
    //     Commands.sequence(
    //         runOnce(
    //             () -> {
    //               lastState = state;
    //               setState(FeederState.UNJAM);
    //             }),
    //         Commands.waitSeconds(0.5),
    //         runOnce(() -> setState(lastState))));
    setDefaultCommand(setVelocity(() -> state.getFeederVelocity().get()));
  }
}
