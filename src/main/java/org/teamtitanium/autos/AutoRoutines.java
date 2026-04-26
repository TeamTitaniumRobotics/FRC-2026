package org.teamtitanium.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
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
    final AutoRoutine routine = factory.newRoutine("Right Outpost Auto");
    Path[] paths = new Path[] {Path.RTS_RFME, Path.RFME_RTSB, Path.RTSB_ROB, Path.ROB_RBB};
    Command autoCmd = resetPose(paths[0]);

    for (Path path : paths) {
      autoCmd = autoCmd.andThen(followPath(path, routine));
    }

    autoCmd =
        autoCmd.andThen(
            Commands.parallel(
                setAutoScore(true),
                Commands.repeatingSequence(
                    setAutoIntakeOverride(true),
                    Commands.waitSeconds(0.9),
                    setAutoIntakeOverride(false),
                    Commands.waitSeconds(0.75))));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  public Command rightDoubleBump() {
    final AutoRoutine routine = factory.newRoutine("Right Double Bump");
    Command autoCmd =
        Commands.sequence(
            resetPose(Path.RTS_RFME_RBSH),
            followPath(Path.RTS_RFME_RBSH, routine),
            followPath(Path.RBSH_RTS, routine),
            followPath(Path.RTS_RFME_RBSH, routine),
            followPath(Path.RBSH_RTS, routine));
    routine.active().onTrue(autoCmd);
    return routine.cmd();
  }

  // public Command rightDoublePassBump() {
  //   final AutoRoutine routine = factory.newRoutine("Right Double Pass Bump");
  //   Command autoCmd = resetPose(Path.RTSR_RFME);

  //   autoCmd =
  //       autoCmd.andThen(
  //           Commands.sequence(
  //               followPath(Path.RTSR_RFME, routine), followPath(Path.RFME_RBSH, routine)));

  //   autoCmd =
  //       autoCmd.andThen(
  //           Commands.deadline(
  //               followPath(Path.RBSH_RTSR, routine),
  //               Commands.waitSeconds(1.0).andThen(shuffleShoot())));

  //   autoCmd =
  //       autoCmd.andThen(
  //           Commands.sequence(
  //               followPath(Path.RTSR_RCME, routine), followPath(Path.RCME_RBSH, routine)));

  //   autoCmd =
  //       autoCmd.andThen(
  //           Commands.deadline(
  //               followPath(Path.RBSH_RTSR, routine),
  //               Commands.waitSeconds(1.0).andThen(shuffleShoot())));

  //   routine.active().onTrue(autoCmd);

  //   return routine.cmd();
  // }

  public Command rightDoublePass() {
    final AutoRoutine routine = factory.newRoutine("Right Double Pass");

    Command autoCmd =
        Commands.sequence(
            resetPose(Path.RTS_RFME),
            followPath(Path.RTS_RFME, routine),
            followPath(Path.RFME_RTSH, routine),
            shuffleShoot().withTimeout(4.0),
            followPath(Path.RTSH_RCME, routine),
            followPath(Path.RCME_RTSH, routine),
            shuffleShoot().withTimeout(4.0));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  public Command leftDoublePass() {
    final AutoRoutine routine = factory.newRoutine("Left Double Pass Auto");

    Command autoCmd =
        Commands.sequence(
            resetPose(Path.LTS_LFME),
            followPath(Path.LTS_LFME, routine),
            followPath(Path.LFME_LTSH, routine),
            shuffleShoot().withTimeout(4.0),
            followPath(Path.LTSH_LCME, routine),
            followPath(Path.LCME_LTSH, routine),
            shuffleShoot().withTimeout(4.0));
    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  private Command followPath(Paths.Path path) {
    return switch (path.action()) {
      case INTAKE -> intakePath(path);
      case SCORE -> scorePath(path);
      case NOTHING -> emptyPath(path);
      default -> emptyPath(path);
    };
  }

  /** Deprecated version of followPath that uses the old auto system. */
  @Deprecated(since = "2026-4-24", forRemoval = true)
  private Command followPath(Path path, AutoRoutine routine) {
    PathAction action = path.action;
    switch (action) {
      case INTAKE:
        return intakePath(path, routine);
      case SCORE:
        return scorePath(path, routine);
      case SHUFFLE:
        return emptyPath(path, routine).deadlineFor(shuffleShoot());
      case NOTHING:
        return emptyPath(path, routine);
      default:
        return emptyPath(path, routine);
    }
  }

  /**
   * Follows a path while running the intake, then turns off the intake at the end.
   *
   * @param path The {@link Paths.Path} to follow while intaking.
   * @return A command that follows the path while intaking, then turns off the intake at the end.
   */
  private Command intakePath(Paths.Path path) {
    return Commands.deadline(emptyPath(path), setAutoIntake(true), setAutoScore(false))
        .andThen(setAutoIntake(false));
  }

  /** Deprecated version of intakePath that uses the old auto system. */
  @Deprecated(since = "2026-4-24", forRemoval = true)
  private Command intakePath(Path path, AutoRoutine routine) {
    return Commands.deadline(path.getPathCommand(), setAutoIntake(true), setAutoScore(false))
        .andThen(setAutoIntake(false));
  }

  /**
   * Follows a path while running the scoring command, then turns off the scoring command at the
   * end.
   *
   * @param path The {@link Paths.Path} to follow while scoring.
   * @return A command that follows the path while scoring, then turns off the scoring command at
   *     the end.
   */
  private Command scorePath(Paths.Path path) {
    return Commands.deadline(emptyPath(path), setAutoIntake(false), setAutoScore(true))
        .andThen(setAutoScore(false));
  }

  /** Deprecated version of scorePath that uses the old auto system. */
  @Deprecated(since = "2026-4-24", forRemoval = true)
  private Command scorePath(Path path, AutoRoutine routine) {
    boolean deployIntake = path.intakeDeployed;
    return Commands.deadline(
            path.getPathCommand(), setAutoScore(true), setAutoIntakeOverride(deployIntake))
        .andThen(setAutoScore(false), setAutoIntakeOverride(false));
  }

  /**
   * Follows a path without running any superstructure commands.
   *
   * @param path The {@link Paths.Path} to follow.
   * @return A command that follows the path without running any superstructure commands.
   */
  private Command emptyPath(Paths.Path path) {
    return swerve.pathBuilder.build(path.getPath());
  }

  /** Deprecated version of emptyPath that uses the old auto system. */
  @Deprecated(since = "2026-4-24", forRemoval = true)
  private Command emptyPath(Path path, AutoRoutine routine) {
    return path.getPathCommand();
  }

  /** Deprecated version of resetPose that uses the old auto system. */
  @Deprecated(since = "2026-4-24", forRemoval = true)
  private Command resetPose(Path path) {
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("Autos/CurrentState", "Resetting Pose");
          RobotState.getInstance()
              .setEstimatedPose(
                  AllianceFlipUtil.apply(
                      path.getPathPlannerPath().getStartingHolonomicPose().orElse(Pose2d.kZero)));
          Logger.recordOutput("Autos/CurrentState", "Pose Reset");
        });
  }

  /**
   * Resets the robot's pose to the starting pose of the given path.
   *
   * @param path The {@link Paths.Path} whose starting pose to reset to.
   * @return A command that resets the robot's pose to the starting pose of the given path.
   */
  private Command resetPose(Paths.Path path) {
    return Commands.runOnce(
        () -> {
          Logger.recordOutput("Autos/CurrentState", "Resetting Pose");
          RobotState.getInstance()
              .setEstimatedPose(AllianceFlipUtil.apply(path.points().get(0).getPoint().getPose()));
          Logger.recordOutput("Autos/CurrentState", "Pose Reset");
        });
  }

  private Command shuffleShoot() {
    return Commands.sequence(setAutoScore(true), shuffleIntake(0.5, 0.75), setAutoScore(false));
  }

  private Command shuffleIntake(double outDelay, double inDelay) {
    return Commands.repeatingSequence(
            setAutoIntakeOverride(true),
            Commands.waitSeconds(outDelay),
            setAutoIntakeOverride(false),
            Commands.waitSeconds(inDelay))
        .finallyDo(interrupted -> autoIntakeOverride = false);
  }

  private Command setAutoIntake(boolean value) {
    return Commands.runOnce(
        () -> {
          autoIntake = value;
          Logger.recordOutput("Autos/CurrentState", "Auto Intake: " + value);
        });
  }

  private Command setAutoIntakeOverride(boolean value) {
    return Commands.runOnce(
        () -> {
          autoIntakeOverride = value;
          Logger.recordOutput("Autos/CurrentState", "Auto Intake Override: " + value);
        });
  }

  private Command setAutoScore(boolean value) {
    return Commands.runOnce(
        () -> {
          autoScore = value;
          Logger.recordOutput("Autos/CurrentState", "Auto Score: " + value);
        });
  }

  public enum PathAction {
    INTAKE,
    SCORE,
    SHUFFLE,
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
  /** Deprecated Path system to be replaced with {@link Paths} */
  @Deprecated(since = "2026-4-24", forRemoval = true)
  public enum Path {
    RTS_RFME(PathAction.INTAKE),
    RTS_RCME(PathAction.INTAKE),
    RTS_RB(PathAction.NOTHING, true),
    RTS_RFME_RBSH(PathAction.INTAKE),
    RTSH_RCME(PathAction.INTAKE),
    RTSR_RFME(PathAction.INTAKE),
    RTSR_RCME(PathAction.INTAKE),
    RFME_RTS(PathAction.NOTHING, true),
    RFME_RTSH(PathAction.NOTHING, true),
    RFME_RBSH(PathAction.NOTHING, true),
    RFME_RTSB(PathAction.NOTHING),
    RCME_RBSH(PathAction.NOTHING, true),
    RCME_RTSH(PathAction.NOTHING, true),
    RTSB_RTSHB(PathAction.NOTHING, true),
    RTSB_ROB(PathAction.INTAKE),
    RTSHB_ROB(PathAction.INTAKE),
    RCME_RTS(PathAction.NOTHING, true),
    ROB_RBB(PathAction.NOTHING, true),
    RB_RTS(PathAction.INTAKE),
    RBSH_RTS(PathAction.SHUFFLE),
    LTS_LFME(PathAction.INTAKE),
    LTSH_LCME(PathAction.INTAKE),
    LFME_LTS(PathAction.NOTHING, true),
    LFME_LTSH(PathAction.NOTHING, true),
    LTS_LB(PathAction.SCORE, true),
    LB_LTS(PathAction.INTAKE),
    LTS_LCME(PathAction.INTAKE),
    LCME_LTS(PathAction.NOTHING, true),
    LCME_LTSH(PathAction.NOTHING, true);

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

    public Command getPathCommand() {
      try {
        return Commands.sequence(
            Commands.runOnce(
                () ->
                    Logger.recordOutput("Autos/CurrentState", "Starting Path: " + this.toString())),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(this.toString())),
            Commands.runOnce(
                () ->
                    Logger.recordOutput("Autos/CurrentState", "Ending Path: " + this.toString())));
      } catch (Exception e) {
        DriverStation.reportError(
            "Failed to load PathPlanner path for auto '" + this.toString() + "'.",
            e.getStackTrace());
        Logger.recordOutput(
            "AutoRoutines/PathLoadError",
            "Failed to load PathPlanner path for auto '" + this.toString() + "': " + e);
        return Commands.none();
      }
    }

    public PathPlannerPath getPathPlannerPath() {
      try {
        return PathPlannerPath.fromPathFile(this.toString());
      } catch (Exception e) {
        DriverStation.reportError(
            "Failed to load PathPlanner path for auto '" + this.toString() + "'.",
            e.getStackTrace());
        Logger.recordOutput(
            "AutoRoutines/PathLoadError",
            "Failed to load PathPlanner path for auto '" + this.toString() + "': " + e);

        Pose2d fallbackPose = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
        Translation2d fallbackTranslation = fallbackPose.getTranslation();
        return new PathPlannerPath(
            List.of(new Waypoint(fallbackTranslation, fallbackTranslation, fallbackTranslation)),
            PathConstraints.unlimitedConstraints(12.0),
            null,
            new GoalEndState(0.0, fallbackPose.getRotation()));
      }
    }
  }
}
