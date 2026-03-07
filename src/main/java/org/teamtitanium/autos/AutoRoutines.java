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
  private static boolean autoScore;

  @AutoLogOutput(key = "Superstructure/AutoIntake")
  public static Trigger autoIntakeReq =
      new Trigger(() -> autoIntake).and(DriverStation::isAutonomous);

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

  public Command getOutpostAuto() {
    final AutoRoutine routine = factory.newRoutine("Outpost Score Auto");
    Path[] paths = new Path[] {Path.RTS_RFS, Path.RFS_RFE, Path.RFE_RTSB, Path.RTSB_ROB};
    Command autoCmd = paths[0].getTrajectory(routine).resetOdometry();

    for (Path path : paths) {
      autoCmd = autoCmd.andThen(followPath(path, routine));
    }

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
        Commands.runOnce(() -> autoIntake = true),
        Commands.runOnce(() -> autoScore = false),
        trajectory.cmd().until(trajectory.done()),
        Commands.runOnce(() -> autoIntake = false));
  }

  private Command scorePath(Path path, AutoRoutine routine) {
    AutoTrajectory trajectory = path.getTrajectory(routine);
    boolean deployIntake = path.intakeDeployed;
    return Commands.sequence(
        Commands.runOnce(() -> autoScore = true),
        Commands.runOnce(() -> autoIntake = deployIntake),
        trajectory.cmd().until(trajectory.done()),
        Commands.runOnce(() -> autoScore = false),
        Commands.runOnce(() -> autoIntake = false));
  }

  private Command emptyPath(Path path, AutoRoutine routine) {
    AutoTrajectory trajectory = path.getTrajectory(routine);
    return trajectory.cmd().until(trajectory.done());
  }

  public enum PathAction {
    INTAKE,
    SCORE,
    NOTHING
  }

  /*
   * Naming Scheme:
   *
   * - First letter: Starting side (R = Right, L = Left)
   * - Second letter: Starting position (T = Trench, B = Bump, H = Hub, O = Outpost)
   *
   * - Third letter: Ending side (R = Right, L = Left)
   * - Fourth letter: Ending position (T = Trench, B = Bump, H = Hub, O = Outpost)
   * - Fifth letter (if present): Final pose (F = Facing forward, B = Facing backward)
   */
  public enum Path {
    RTS_RFS(PathAction.INTAKE),
    RFS_RFE(PathAction.INTAKE),
    RFE_RTSB(PathAction.NOTHING),
    RTSB_ROB(PathAction.SCORE, true);

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
