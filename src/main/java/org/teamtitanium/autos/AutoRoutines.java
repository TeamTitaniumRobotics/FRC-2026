package org.teamtitanium.autos;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import org.teamtitanium.RobotState;
import org.teamtitanium.subsystems.feeder.Feeder;
import org.teamtitanium.subsystems.intake.Intake;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel;
import org.teamtitanium.subsystems.shooter.hood.Hood;
import org.teamtitanium.subsystems.shooter.turret.Turret;
import org.teamtitanium.subsystems.spindexer.Spindexer;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.AllianceFlipUtil;

public class AutoRoutines {

  private final Swerve swerve;
  @Getter private final AutoFactory factory;
  private final RobotState robotState = RobotState.getInstance();
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;
  private final Intake intake;
  private final Feeder feeder;
  private final Spindexer spindexer;

  public AutoRoutines(
      Swerve swerve,
      TrajectoryLogger<SwerveSample> trajLogger,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Intake intake,
      Feeder feeder,
      Spindexer spindexer) {
    this.turret = turret;
    this.swerve = swerve;
    this.hood = hood;
    this.flywheel = flywheel;
    this.intake = intake;
    this.feeder = feeder;
    this.spindexer = spindexer;
    this.factory =
        new AutoFactory(
            robotState::getEstimatedPose, // A function that returns the current robot pose
            robotState
                ::setEstimatedPose, // A function that resets the current robot pose to the provided
            // Pose2d
            this.swerve::followChoreoTrajectory, // The drive subsystem trajectory follower
            AllianceFlipUtil.shouldFlip(), // If alliance flipping should be enabled
            this.swerve, // The drive subsystem
            trajLogger);
  }

  // public AutoRoutine exampleAutoRoutine() {
  //   AutoRoutine routine = factory.newRoutine("exampleRoutine"); // Name the routine
  //   AutoTrajectory driveSomewhere = routine.trajectory("NewPath"); // Load the trajectory

  //   routine
  //       .active()
  //       .onTrue(Commands.sequence(driveSomewhere.resetOdometry(), driveSomewhere.cmd()));

  //   return routine;
  // }

  public AutoRoutine idealRoutine() {
    AutoRoutine routine = factory.newRoutine("idealRoutine");
    AutoTrajectory idealPaths = routine.trajectory("Ideal");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                idealPaths.resetOdometry(),
                Commands.print("This is the start of the routine."),
                idealPaths.cmd()));
    idealPaths.atTime("Intake").onTrue(intake.intake().andThen(Commands.print("Intake starting")));
    idealPaths.atTime("Stow").onTrue(intake.stow().andThen(Commands.print("Intake stopping")));
    // // long-running command: start intake repeatedly, stow on end
    // WrapperCommand intakeBetweenMarkers =
    //     Commands.sequence(
    //             // print once when the intake command starts
    //             Commands.runOnce(() -> System.out.println("Intake starting")),
    //             // run intake while scheduled; stow and print when it ends
    //             Commands.runEnd(
    //                 () -> intake.intake(),
    //                 () -> {
    //                   intake.stow();
    //                   System.out.println("Intake stopping");
    //                 }))
    //         .withName("IntakeBetweenMarkers");

    // routine
    //     .active()
    //     .onTrue(
    //         Commands.sequence(
    //             idealPaths.resetOdometry(),
    //             Commands.print("This is the start of the routine."),
    //             idealPaths.cmd()));
    // // start the intake when you hit the StartMarker
    // idealPaths.atTime("IntakeStartMarker").onTrue(intakeBetweenMarkers);

    // // stop the intake when you hit the EndMarker by cancelling the command
    // idealPaths
    //     .atTime("IntakeEndMarker")
    //     .onTrue(
    //         Commands.runOnce(() -> CommandScheduler.getInstance().cancel(intakeBetweenMarkers)));
    return routine;
  }

  public AutoRoutine movementRoutine() {
    AutoRoutine routine = factory.newRoutine("movementRoutine");
    AutoTrajectory testPath = routine.trajectory("TestPath");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                testPath.resetOdometry(),
                Commands.print("This is the start of the routine."),
                testPath.cmd()));
    return routine;
  }

  public AutoRoutine rotateRoutine() {
    AutoRoutine routine = factory.newRoutine("rotateRoutine");
    AutoTrajectory rotateInPlace = routine.trajectory("RotateInPlace");
    routine
        .active()
        .onTrue(
            Commands.sequence(
                rotateInPlace.resetOdometry(),
                Commands.print("This is the start of the routine."),
                rotateInPlace.cmd()));
    return routine;
  }
}
