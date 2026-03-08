package org.teamtitanium.subsystems;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.feeder.Feeder;
import org.teamtitanium.subsystems.feeder.Feeder.FeederState;
import org.teamtitanium.subsystems.intake.Intake;
import org.teamtitanium.subsystems.intake.Intake.IntakeState;
import org.teamtitanium.subsystems.shooter.Shooter;
import org.teamtitanium.subsystems.shooter.Shooter.ShooterState;
import org.teamtitanium.subsystems.shooter.turret.TurretConstants;
import org.teamtitanium.subsystems.spindexer.Spindexer;
import org.teamtitanium.subsystems.spindexer.Spindexer.SpindexerState;
import org.teamtitanium.utils.virtualsubsystem.VirtualSubsystem;

/**
 * Coordinates all subsystem groups via a lean game-intent state machine. Each subsystem has its own
 * state enum; this class translates (game state + modifiers) into subsystem states through a single
 * {@link #applySubStates()} resolution function.
 */
public class Superstructure extends VirtualSubsystem {
  // ───────────────────────────── Game-level state ─────────────────────────────

  /**
   * Represents the high-level game intent of the robot. Subsystem-specific behaviors are resolved
   * independently — this enum should NOT encode per-subsystem combinations.
   */
  public enum SuperstructureState {
    IDLE,
    INTAKE,
    SPIN_UP_SCORE,
    SCORE,
    SPIN_UP_PASS,
    PASS,
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

  // ----------------------------- Subsystems -----------------------------------

  private final Shooter shooter;
  private final Feeder feeder;
  private final Spindexer spindexer;
  private final Intake intake;

  // ----------------------------- Driver inputs --------------------------------

  private final Trigger intakeReq;
  private final Trigger scoreReq;
  private final Trigger passReq;
  private final Trigger spitReq;
  private final Trigger stowReq;
  private final Trigger hasFuel;

  // ----------------------------- Modifier triggers --------------------------------

  // These are orthogonal to the game state and persist across state transitions
  // unless explicitly reset inside setState().

  /** When active, the intake rack stays deployed regardless of game state. */
  @AutoLogOutput(key = "Superstructure/IntakeDeployed")
  @Getter
  private final Trigger intakeDeployed;

  private boolean intakeDeployedValue = false;

  /** When active, the hood will auto-stow (e.g. going under the trench). */
  @AutoLogOutput(key = "Superstructure/TrenchStowOverride")
  @Getter
  private final Trigger trenchStowOverride;

  // ----------------------------- Construction --------------------------------

  public Superstructure(
      Shooter shooter,
      Feeder feeder,
      Spindexer spindexer,
      Intake intake,
      Trigger trenchStowOverride,
      CommandXboxController driver) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.spindexer = spindexer;
    this.intake = intake;

    intakeReq = driver.leftTrigger();
    scoreReq = driver.rightTrigger();
    passReq = new Trigger(() -> false);
    stowReq = new Trigger(() -> false);
    spitReq = new Trigger(() -> false);

    hasFuel = spindexer.hasFuel.or(feeder.hasFuel);

    // Modifier triggers — backed by simple booleans toggled via commands
    intakeDeployed = new Trigger(() -> intakeDeployedValue);
    this.trenchStowOverride = trenchStowOverride;

    // Pass the hood-stow override trigger into Shooter so it can respect it
    shooter.setHoodStowOverride(this.trenchStowOverride);

    bindTransitions();
    bindModifierToggles(driver);
  }

  @Override
  public void periodic() {
    applySubStates();
  }

  @Override
  public void simulationPeriodic() {
    Pose3d turretPose =
        new Pose3d(
            TurretConstants.TURRET_TO_ROBOT.getTranslation(),
            new Rotation3d(0.0, 0.0, shooter.getTurretAngle().in(Radians)));
    Logger.recordOutput("Superstructure/TurretPose", turretPose);
  }

  // ----------------------------- State transitions --------------------------------

  private void bindTransitions() {
    // IDLE <--> INTAKE
    bindTransition(SuperstructureState.IDLE, SuperstructureState.INTAKE, intakeReq);
    bindTransition(SuperstructureState.INTAKE, SuperstructureState.IDLE, intakeReq.negate());

    bindTransition(SuperstructureState.INTAKE, SuperstructureState.IDLE, stowReq);
    bindTransition(SuperstructureState.SPIN_UP_SCORE, SuperstructureState.IDLE, stowReq);
    bindTransition(SuperstructureState.SCORE, SuperstructureState.IDLE, stowReq);

    // IDLE -> SPIN_UP_SCORE -> SCORE
    bindTransition(SuperstructureState.IDLE, SuperstructureState.SPIN_UP_SCORE, scoreReq);
    bindTransition(
        SuperstructureState.SPIN_UP_SCORE,
        SuperstructureState.SCORE,
        shooter.atSetpoint().and(scoreReq));
    bindTransition(
        SuperstructureState.SCORE,
        SuperstructureState.SPIN_UP_SCORE,
        scoreReq.and(shooter.atSetpoint().negate().debounce(0.25)));
    bindTransition(SuperstructureState.SPIN_UP_SCORE, SuperstructureState.IDLE, scoreReq.negate());
    bindTransition(SuperstructureState.SCORE, SuperstructureState.IDLE, scoreReq.negate());

    // IDLE -> SPIN_UP_PASS -> PASS
    bindTransition(SuperstructureState.IDLE, SuperstructureState.SPIN_UP_PASS, passReq);
    bindTransition(
        SuperstructureState.SPIN_UP_PASS,
        SuperstructureState.PASS,
        shooter.atSetpoint().and(passReq));
    bindTransition(
        SuperstructureState.PASS,
        SuperstructureState.IDLE,
        passReq.negate().and(() -> stateTimer.hasElapsed(0.5)));
    bindTransition(
        SuperstructureState.SPIN_UP_PASS,
        SuperstructureState.IDLE,
        passReq.negate().and(() -> stateTimer.hasElapsed(0.5)));

    // IDLE -> EJECT (spit fuel out the front)
    bindTransition(SuperstructureState.IDLE, SuperstructureState.EJECT, spitReq);
    // Return to the state we were in before ejecting
    SuperstructureState.EJECT
        .getTrigger()
        .and(spitReq.negate())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      setState(previousState);
                    })
                .ignoringDisable(true)
                .withName("ReturnFromEject"));
  }

  private void bindTransition(
      SuperstructureState from, SuperstructureState to, Trigger transitionTrigger) {
    from.getTrigger().and(transitionTrigger).onTrue(setStateCommand(to));
  }

  // ----------------------------- Modifier toggle bindings --------------------------------

  private void bindModifierToggles(CommandXboxController driver) {
    // Toggle intake deployed on left bumper press
    driver
        .leftBumper()
        .onTrue(
            Commands.runOnce(() -> intakeDeployedValue = !intakeDeployedValue)
                .ignoringDisable(true)
                .withName("ToggleIntakeDeployed"));
  }

  // -------------- Central resolution: (game state + modifiers) -> sub states --------------

  /**
   * Returns a command that should be scheduled once and kept running for the lifetime of
   * teleop/auto. Every loop iteration it reads the current {@link SuperstructureState} plus
   * modifier triggers and pushes the resolved sub-state into each subsystem.
   */
  public void applySubStates() {
    // --- Resolve Shooter state ---
    switch (state) {
      case IDLE -> shooter.setState(ShooterState.STOW);
      case EJECT -> shooter.setState(ShooterState.EJECT);
      default -> shooter.setState(ShooterState.AIM);
    }

    // --- Resolve Intake state ---
    switch (state) {
      case INTAKE -> intake.setState(IntakeState.INTAKE);
      case EJECT -> intake.setState(IntakeState.EJECT);
      case SPIN_UP_SCORE, SPIN_UP_PASS -> {
        // If intake deployed override is active, keep intaking; else agitate
        intake.setState(intakeDeployed.getAsBoolean() ? IntakeState.INTAKE : IntakeState.AGITATE);
      }
      case SCORE, PASS -> {
        // If intake deployed override is active, keep intaking; else stow
        intake.setState(intakeDeployed.getAsBoolean() ? IntakeState.INTAKE : IntakeState.AGITATE);
      }
      default -> {
        // IDLE / PREPPED / CLIMB states
        intake.setState(intakeDeployed.getAsBoolean() ? IntakeState.INTAKE : IntakeState.STOW);
      }
    }

    // --- Resolve Feeder state ---
    switch (state) {
      case SCORE, PASS, EJECT -> feeder.setState(FeederState.FEED);
      default -> feeder.setState(FeederState.IDLE);
    }

    // --- Resolve Spindexer state ---
    switch (state) {
      case INTAKE, SPIN_UP_SCORE, SPIN_UP_PASS -> spindexer.setState(SpindexerState.IDLE);
      case SCORE, PASS, EJECT -> spindexer.setState(SpindexerState.FEED);
      default -> spindexer.setState(SpindexerState.IDLE);
    }
  }

  // ----------------------------- setState helpers --------------------------------

  /**
   * Sets the game state and optionally resets modifiers atomically.
   *
   * @param newState the new game state
   * @param modifierResets optional runnables that reset modifier values on this transition
   */
  private void setState(SuperstructureState newState, Runnable... modifierResets) {
    Logger.recordOutput("Superstructure/StateTransition", state.name() + " -> " + newState.name());
    this.previousState = Superstructure.state;
    Superstructure.state = newState;
    stateTimer.restart();

    for (Runnable reset : modifierResets) {
      reset.run();
    }
  }

  /**
   * Returns a command that performs a state transition, for use with Trigger.onTrue().
   *
   * @param newState the new game state
   * @param modifierResets optional runnables that reset modifier values on this transition
   */
  private Command setStateCommand(SuperstructureState newState, Runnable... modifierResets) {
    return Commands.runOnce(() -> setState(newState, modifierResets))
        .ignoringDisable(true)
        .withName("SetSuperstructureState(" + newState + ")");
  }
}
