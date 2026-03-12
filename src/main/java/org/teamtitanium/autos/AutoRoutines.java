package org.teamtitanium.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.autos.generated.ChoreoTraj;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.AllianceFlipUtil;

public class AutoRoutines {
  private final Swerve swerve;
  private final AutoFactory factory;
  private final RobotState robotState = RobotState.getInstance();

  private static boolean autoIntake;
  private static boolean autoIntakeOverride;
  private static boolean autoScore;

  @AutoLogOutput(key = "Superstructure/AutoIntake")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/AutoIntakeOverride")
  public static Trigger autoIntakeOverrideReq =
      new Trigger(() -> autoIntakeOverride).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/AutoScore")
  public static Trigger autoScoreReq =
      new Trigger(() -> autoScore).and(DriverStation::isAutonomous);

  public AutoRoutines(Swerve swerve) {
    this.swerve = swerve;
    this.factory =
        new AutoFactory(
            robotState::getEstimatedPose,
            robotState::setEstimatedPose,
            this.swerve::followChoreoTrajectory,
            AllianceFlipUtil.shouldFlip(),
            this.swerve,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Choreo/ActiveTraj",
                  AllianceFlipUtil.shouldFlip() ? traj.flipped().getPoses() : traj.getPoses());
            });
  }

  public Command straightTuningAuto() {
    final AutoRoutine routine = factory.newRoutine("Straight Tuning Auto");
    var trajectory = ChoreoTraj.StraightPath.asAutoTraj(routine);

    Command autoCmd =
        Commands.sequence(trajectory.resetOdometry(), Commands.waitSeconds(1.0), trajectory.cmd());

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  public Command getRightOutpostAuto() {
    final AutoRoutine routine = factory.newRoutine("Right Outpost Score Auto");
    Path[] paths = new Path[] {Path.RTS_RFME, Path.RFME_RTSB, Path.RTSB_ROB};
    Command autoCmd = paths[0].getTrajectory(routine).resetOdometry();

    for (Path path : paths) {
      autoCmd = autoCmd.andThen(followPath(path, routine));
    }

    autoCmd = autoCmd.andThen(Commands.sequence(setAutoIntake(true), setAutoScore(true)));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  public Command leftDoublePass() {
    final AutoRoutine routine = factory.newRoutine("Left Double Pass Auto");
    Path[] paths = new Path[] {Path.LTS_LFME, Path.LFME_LTSB, Path.LTSB_LOB};
    Command autoCmd = paths[0].getTrajectory(routine).resetOdometry();

    for (Path path : paths) {
      autoCmd = autoCmd.andThen(followPath(path, routine));
    }

    autoCmd = autoCmd.andThen(setAutoScore(true));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  private Command followPath(Path path, AutoRoutine routine) {
    PathAction action = path.action;
    switch (action) {
      case INTAKE:
        return intakePath(path, routine);
      case SCORE:
        return scorePath(path, routine);
      case NOTHING:
        return emptyPath(path, routine);
      default:
        return emptyPath(path, routine);
    }
  }

  private Command intakePath(Path path, AutoRoutine routine) {
    AutoTrajectory trajectory = path.getTrajectory(routine);
    return Commands.sequence(
        setAutoIntake(true),
        setAutoScore(false),
        trajectory.cmd().until(trajectory.done()),
        setAutoIntake(false));
  }

  private Command scorePath(Path path, AutoRoutine routine) {
    AutoTrajectory trajectory = path.getTrajectory(routine);
    boolean deployIntake = path.intakeDeployed;
    return Commands.sequence(
        setAutoScore(true),
        setAutoIntakeOverride(deployIntake),
        trajectory.cmd().until(trajectory.done()),
        setAutoScore(false),
        setAutoIntakeOverride(false));
  }

  private Command emptyPath(Path path, AutoRoutine routine) {
    AutoTrajectory trajectory = path.getTrajectory(routine);
    return trajectory.cmd().until(trajectory.done());
  }

  private Command setAutoIntake(boolean value) {
    return Commands.runOnce(() -> autoIntake = value);
  }

  private Command setAutoIntakeOverride(boolean value) {
    return Commands.runOnce(() -> autoIntakeOverride = value);
  }

  private Command setAutoScore(boolean value) {
    return Commands.runOnce(() -> autoScore = value);
  }

  public enum PathAction {
    INTAKE,
    SCORE,
    NOTHING
  }

  /*
   * Naming Scheme:
   * First Letter Chain
   * - First letter: Starting side (R = Right, L = Left)
   * - Second letter (if followed by M): (C = Close, M = Middle, F = Far)
   * - Second/Third letter: Starting position (T = Trench, B = Bump, M = Middle, H = Hub, O = Outpost)
   * - Third/Fourth letter (if present): (S = Start, E = End)
   * - Third/Fourth letter (if present): Starting rotation (F = Facing forward, B = Facing backward)
   *
   * Second Letter Chain
   * - First letter: Ending side (R = Right, L = Left)
   * - Second letter (if followed by M): (C = Close, M = Middle, F = Far)
   * - Second/Third letter: Ending position (T = Trench, B = Bump, M = Middle, H = Hub, O = Outpost)
   * - Third/Fourth letter (if present): (S = Start, E = End)
   * - Third/Fourth letter (if present): Final pose (F = Facing forward, B = Facing backward)
   */
  public enum Path {
    RTS_RFME(PathAction.INTAKE),
    RFME_RTSB(PathAction.NOTHING),
    RTSB_ROB(PathAction.SCORE, true),
    LTS_LFME(PathAction.INTAKE),
    LFME_LTSB(PathAction.NOTHING),
    LTSB_LOB(PathAction.NOTHING);

    private final PathAction action;
    private final boolean intakeDeployed;

    private Path(PathAction action) {
      this.action = action;
      this.intakeDeployed = action == PathAction.INTAKE;
    }

    private Path(PathAction action, boolean intakeDeployed) {
      this.action = action;
      this.intakeDeployed = intakeDeployed;
    }

    public AutoTrajectory getTrajectory(AutoRoutine routine) {
      return ChoreoTraj.ALL_TRAJECTORIES.get(this.name()).asAutoTraj(routine);
    }
  }
}
