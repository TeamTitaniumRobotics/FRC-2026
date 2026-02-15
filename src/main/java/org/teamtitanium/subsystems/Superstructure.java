package org.teamtitanium.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.feeder.Feeder;
import org.teamtitanium.subsystems.intake.Intake;
import org.teamtitanium.subsystems.shooter.Shooter;
import org.teamtitanium.subsystems.spindexer.Spindexer;

public class Superstructure {
  public enum SuperstructureState {
    IDLE,
    INTAKE,
    PREPPED,
    SPIN_UP_SCORE,
    SCORE,
    SCORE_THROUGH,
    SPIN_UP_PASS,
    PASS,
    PASS_THROUGH,
    PREP_HUB,
    SCORE_HUB,
    PREP_OUTPOST,
    SCORE_OUTPOST,
    EJECT,
    PREP_CLIMB,
    CLIMB,
    CLIMB_L1,
    DE_CLIMB_L1;

    @Getter private final Trigger trigger;

    private SuperstructureState() {
      trigger = new Trigger(() -> state == this);
    }
  }

  @AutoLogOutput(key = "Superstructure/State")
  @Getter
  private static SuperstructureState state = SuperstructureState.IDLE;

  private SuperstructureState previousState = SuperstructureState.IDLE;

  private final Timer stateTimer = new Timer();

  private final Shooter shooter;
  private final Feeder feeder;
  private final Spindexer spindexer;
  private final Intake intake;

  private final Trigger intakeReq;
  private final Trigger scoreReq;
  private final Trigger spitReq;
  private Trigger hasFuel = new Trigger(() -> true); // TODO: Replace with spindexer fuel sensor

  public Superstructure(
      Shooter shooter,
      Feeder feeder,
      Spindexer spindexer,
      Intake intake,
      CommandXboxController driver) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.spindexer = spindexer;
    this.intake = intake;

    intakeReq = driver.leftTrigger();
    scoreReq = driver.rightTrigger();
    spitReq = driver.back();

    bindTransitions();
    bindStates();
  }

  private void bindTransitions() {
    bindTransition(SuperstructureState.IDLE, SuperstructureState.INTAKE, intakeReq);
    bindTransition(SuperstructureState.INTAKE, SuperstructureState.IDLE, intakeReq.negate());

    bindTransition(
        hasFuel.and(intakeReq.negate()),
        SuperstructureState.PREPPED,
        SuperstructureState.IDLE,
        SuperstructureState.INTAKE);
    bindTransition(
        SuperstructureState.PREPPED,
        SuperstructureState.IDLE,
        hasFuel.negate().and(() -> stateTimer.hasElapsed(0.5)));

    bindTransition(SuperstructureState.PREPPED, SuperstructureState.SPIN_UP_SCORE, scoreReq);
    bindTransition(
        SuperstructureState.SPIN_UP_SCORE,
        SuperstructureState.SCORE,
        shooter.atSetpoint().and(scoreReq));
    bindTransition(SuperstructureState.SCORE, SuperstructureState.PREPPED, scoreReq.negate());

    bindTransition(SuperstructureState.PREPPED, SuperstructureState.SPIN_UP_PASS, spitReq);
    bindTransition(
        SuperstructureState.SPIN_UP_PASS,
        SuperstructureState.PASS,
        shooter.atSetpoint().and(spitReq));
    bindTransition(
        SuperstructureState.PASS,
        SuperstructureState.PREPPED,
        spitReq.negate().and(() -> stateTimer.hasElapsed(0.5)));

    bindTransition(
        spitReq, SuperstructureState.EJECT, SuperstructureState.IDLE, SuperstructureState.PREPPED);
  }

  private void bindTransition(
      SuperstructureState from, SuperstructureState to, Trigger transitionTrigger) {
    from.getTrigger().and(transitionTrigger).onTrue(setState(to));
  }

  private void bindTransition(
      Trigger transitionTrigger, SuperstructureState to, SuperstructureState... from) {
    for (SuperstructureState state : from) {
      state.getTrigger().and(transitionTrigger).onTrue(setState(to));
    }
  }

  private void bindStates() {
    bindCommands(
        SuperstructureState.IDLE, shooter.stow(), feeder.idle(), spindexer.idle(), intake.stow());

    bindCommands(SuperstructureState.INTAKE, intake.intake(), shooter.aim());

    bindCommands(SuperstructureState.SPIN_UP_SCORE, shooter.aim());

    bindCommands(SuperstructureState.SCORE, shooter.aim());
  }

  private void bindCommands(SuperstructureState state, Command... commands) {
    state.getTrigger().whileTrue(Commands.parallel(commands));
  }

  private Command setState(SuperstructureState newState) {
    return Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "Superstructure/StateTransition", state.name() + " -> " + newState.name());
              this.previousState = Superstructure.state;
              Superstructure.state = newState;
              stateTimer.restart();
            })
        .ignoringDisable(true)
        .withName("SetSuperstructureState(" + newState + ")");
  }
}
