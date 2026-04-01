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
  private static boolean autoSpinUp;
  private static boolean autoScore;

  @AutoLogOutput(key = "Superstructure/AutoIntake")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/AutoIntakeOverride")
  public static Trigger autoIntakeOverrideReq =
      new Trigger(() -> autoIntakeOverride).and(DriverStation::isAutonomous);

  @AutoLogOutput(key = "Superstructure/AutoSpinUp")
  public static Trigger autoSpinUpReq =
      new Trigger(() -> autoSpinUp).and(DriverStation::isAutonomous);

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

  public Command rightDoublePassBump() {
    final AutoRoutine routine = factory.newRoutine("Right Double Pass Bump");
    Command autoCmd = resetPose(Path.RTSR_RFME);

    autoCmd =
        autoCmd.andThen(
            Commands.sequence(
                followPath(Path.RTSR_RFME, routine), followPath(Path.RFME_RBSH, routine)));

    autoCmd =
        autoCmd.andThen(
            Commands.deadline(
                followPath(Path.RBSH_RTSR, routine),
                Commands.waitSeconds(1.0).andThen(shuffleIntake())));

    autoCmd =
        autoCmd.andThen(
            Commands.sequence(
                followPath(Path.RTSR_RCME, routine), followPath(Path.RCME_RBSH, routine)));

    autoCmd =
        autoCmd.andThen(
            Commands.deadline(
                followPath(Path.RBSH_RTSR, routine),
                Commands.waitSeconds(1.0).andThen(shuffleIntake())));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  public Command rightDoublePass() {
    final AutoRoutine routine = factory.newRoutine("Right Double Pass");
    Command autoCmd = resetPose(Path.RTS_RFME);

    autoCmd =
        autoCmd.andThen(
            Commands.sequence(
                followPath(Path.RTS_RFME, routine), followPath(Path.RFME_RTSH, routine)));

    autoCmd =
        autoCmd.andThen(
            Commands.parallel(
                    setAutoScore(true),
                    Commands.repeatingSequence(
                        setAutoIntakeOverride(true),
                        Commands.waitSeconds(0.8),
                        setAutoIntakeOverride(false),
                        Commands.waitSeconds(0.75)))
                .withTimeout(4.5));

    autoCmd =
        autoCmd.andThen(
            Commands.sequence(
                followPath(Path.RTSH_RCME, routine), followPath(Path.RCME_RTSH, routine)));

    autoCmd =
        autoCmd.andThen(
            Commands.parallel(
                    setAutoScore(true),
                    Commands.repeatingSequence(
                        setAutoIntakeOverride(true),
                        Commands.waitSeconds(0.75),
                        setAutoIntakeOverride(false),
                        Commands.waitSeconds(0.75)))
                .withTimeout(4.5));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  public Command leftDoublePass() {
    final AutoRoutine routine = factory.newRoutine("Left Double Pass Auto");
    Path[] paths = new Path[] {Path.LTS_LFME, Path.LFME_LTS, Path.LTS_LB};
    Command autoCmd = resetPose(paths[0]);

    autoCmd =
        autoCmd.andThen(
            Commands.sequence(
                followPath(Path.LTS_LFME, routine),
                followPath(Path.LFME_LTS, routine),
                followPath(Path.LTS_LB, routine)));

    autoCmd =
        autoCmd.andThen(
            Commands.parallel(
                    setAutoScore(true),
                    Commands.repeatingSequence(
                        setAutoIntakeOverride(true),
                        Commands.waitSeconds(0.8),
                        setAutoIntakeOverride(false),
                        Commands.waitSeconds(0.75)))
                .withTimeout(4.5));

    autoCmd =
        autoCmd.andThen(
            Commands.sequence(
                followPath(Path.LB_LTS, routine),
                followPath(Path.LTS_LCME, routine),
                followPath(Path.LCME_LTS, routine),
                followPath(Path.LTS_LB, routine)));

    autoCmd =
        autoCmd.andThen(Commands.parallel(setAutoScore(true), shuffleIntake()).withTimeout(4.5));

    routine.active().onTrue(autoCmd);

    return routine.cmd();
  }

  private Command followPath(Path path, AutoRoutine routine) {
    PathAction action = path.action;
    switch (action) {
      case INTAKE:
        return intakePath(path, routine);
      case SPIN_UP:
        return spinUpPath(path, routine);
      case SCORE:
        return scorePath(path, routine);
      case NOTHING:
        return emptyPath(path, routine);
      default:
        return emptyPath(path, routine);
    }
  }

  private Command intakePath(Path path, AutoRoutine routine) {
    return Commands.sequence(
        setAutoIntake(true), setAutoScore(false), path.getPathCommand(), setAutoIntake(false));
  }

  private Command spinUpPath(Path path, AutoRoutine routine) {
    boolean deployIntake = path.intakeDeployed;
    return Commands.sequence(
        setAutoSpinUp(true),
        setAutoIntakeOverride(deployIntake),
        path.getPathCommand(),
        setAutoIntakeOverride(false));
  }

  private Command scorePath(Path path, AutoRoutine routine) {
    boolean deployIntake = path.intakeDeployed;
    return Commands.sequence(
        setAutoScore(true),
        setAutoSpinUp(false),
        setAutoIntakeOverride(deployIntake),
        path.getPathCommand(),
        setAutoScore(false),
        setAutoIntakeOverride(false));
  }

  private Command emptyPath(Path path, AutoRoutine routine) {
    return path.getPathCommand();
  }

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

  private Command shuffleIntake() {
    return shuffleIntake(0.75, 0.75);
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

  private Command setAutoSpinUp(boolean value) {
    return Commands.runOnce(
        () -> {
          autoSpinUp = value;
          Logger.recordOutput("Autos/CurrentState", "Auto Spin Up: " + value);
        });
  }

  private Command setAutoScore(boolean value) {
    return Commands.runOnce(
        () -> {
          autoScore = value;
          if (value) {
            autoSpinUp = false;
          }
          Logger.recordOutput("Autos/CurrentState", "Auto Score: " + value);
        });
  }

  public enum PathAction {
    INTAKE,
    SPIN_UP,
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
    RTS_RCME(PathAction.INTAKE),
    RTS_RB(PathAction.SPIN_UP, true),
    RTSH_RCME(PathAction.INTAKE),
    RTSR_RFME(PathAction.INTAKE),
    RTSR_RCME(PathAction.INTAKE),
    RFME_RTS(PathAction.NOTHING, true),
    RFME_RTSH(PathAction.SPIN_UP, true),
    RFME_RBSH(PathAction.SPIN_UP, true),
    RFME_RTSB(PathAction.NOTHING),
    RCME_RBSH(PathAction.SPIN_UP, true),
    RCME_RTSH(PathAction.SPIN_UP, true),
    RTSB_RTSHB(PathAction.SPIN_UP, true),
    RTSB_ROB(PathAction.INTAKE),
    RTSHB_ROB(PathAction.INTAKE),
    RCME_RTS(PathAction.NOTHING, true),
    ROB_RBB(PathAction.SPIN_UP, true),
    RB_RTS(PathAction.INTAKE),
    RBSH_RTSR(PathAction.SCORE),
    LTS_LFME(PathAction.INTAKE),
    LFME_LTS(PathAction.NOTHING, true),
    LTS_LB(PathAction.SCORE, true),
    LB_LTS(PathAction.INTAKE),
    LTS_LCME(PathAction.INTAKE),
    LCME_LTS(PathAction.NOTHING, true);

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
