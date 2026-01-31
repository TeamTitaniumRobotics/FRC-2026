package org.teamtitanium.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel.FlywheelState;
import org.teamtitanium.subsystems.shooter.hood.Hood;
import org.teamtitanium.subsystems.shooter.hood.Hood.HoodState;
import org.teamtitanium.subsystems.shooter.turret.Turret;
import org.teamtitanium.subsystems.shooter.turret.Turret.TurretState;

public class Superstructure {
  public enum SuperstructureState {
    IDLE(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    INTAKE(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    PREPPED(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    SPIN_UP_SCORE(FlywheelState.SCORE, HoodState.SHOOT, TurretState.TRACK),
    SCORE(FlywheelState.SCORE, HoodState.SHOOT, TurretState.TRACK),
    SCORE_THROUGH(FlywheelState.SCORE, HoodState.SHOOT, TurretState.TRACK),
    SPIN_UP_PASS(FlywheelState.PASS, HoodState.PASS, TurretState.TRACK),
    PASS(FlywheelState.PASS, HoodState.PASS, TurretState.TRACK),
    PASS_THROUGH(FlywheelState.PASS, HoodState.PASS, TurretState.TRACK),
    PREP_HUB(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    SCORE_HUB(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    PREP_OUTPOST(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    SCORE_OUTPOST(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    EJECT(FlywheelState.IDLE, HoodState.EJECT, TurretState.STOW),
    PREP_CLIMB(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    CLIMB(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    CLIMB_L1(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW),
    DE_CLIMB_L1(FlywheelState.IDLE, HoodState.STOWED, TurretState.STOW);

    @Getter private final FlywheelState flywheelState;
    @Getter private final HoodState hoodState;
    @Getter private final TurretState turretState;
    @Getter private final Trigger trigger;

    private SuperstructureState(
        FlywheelState flywheelState, HoodState hoodState, TurretState turretState) {
      this.flywheelState = flywheelState;
      this.hoodState = hoodState;
      this.turretState = turretState;
      trigger = new Trigger(() -> state == this);
    }
  }

  @AutoLogOutput(key = "Superstructure/State")
  @Getter
  private static SuperstructureState state = SuperstructureState.IDLE;

  private SuperstructureState previousState = SuperstructureState.IDLE;

  private final Timer stateTimer = new Timer();

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  private final Trigger intakeReq;
  private final Trigger scoreReq;
  private final Trigger spitReq;
  private Trigger hasFuel = new Trigger(() -> true); // TODO: Replace with spindexer fuel sensor

  public Superstructure(Turret turret, Hood hood, Flywheel flywheel, CommandXboxController driver) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;

    intakeReq = driver.leftTrigger();
    scoreReq = driver.rightTrigger();
    spitReq = driver.back();

    bindTransitions();
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
        flywheel.atSetpoint().and(hood.atSetpoint()).and(turret.atSetpoint()).and(scoreReq));
    bindTransition(
        SuperstructureState.SCORE,
        SuperstructureState.PREPPED,
        scoreReq.negate().and(() -> stateTimer.hasElapsed(0.5)));

    bindTransition(SuperstructureState.PREPPED, SuperstructureState.SPIN_UP_PASS, spitReq);
    bindTransition(
        SuperstructureState.SPIN_UP_PASS,
        SuperstructureState.PASS,
        flywheel.atSetpoint().and(hood.atSetpoint()).and(turret.atSetpoint()).and(spitReq));
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
    Trigger fromTrigger = transitionTrigger;
    for (SuperstructureState state : from) {
      fromTrigger = fromTrigger.and(state.getTrigger());
    }
    fromTrigger.onTrue(setState(to));
  }

  private void setSubsystemStates() {
    // Only update subsystem states if not in manual override
    if (!flywheel.isManualOverride()) {
      flywheel.setCurrentState(state.getFlywheelState());
    }
    if (!hood.isManualOverride()) {
      hood.setCurrentState(state.getHoodState());
    }
    if (!turret.isManualOverride()) {
      turret.setCurrentState(state.getTurretState());
    }
  }

  private Command setState(SuperstructureState newState) {
    return Commands.runOnce(
            () -> {
              Logger.recordOutput(
                  "Superstructure/StateTransition", state.name() + " -> " + newState.name());
              this.previousState = Superstructure.state;
              Superstructure.state = newState;
              stateTimer.restart();
              setSubsystemStates();
            })
        .ignoringDisable(true)
        .withName("SetSuperstructureState(" + newState + ")");
  }
}
