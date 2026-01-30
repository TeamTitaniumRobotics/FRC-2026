package org.teamtitanium.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import org.teamtitanium.RobotState;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import org.teamtitanium.subsystems.swerve.Swerve;

public class AutoRoutines {

    private final Swerve swerve;
    @Getter private final AutoFactory factory;
    private final RobotState robotState = RobotState.getInstance();

    public AutoRoutines(Swerve swerve) {
        this.swerve = swerve;
        this.factory = new AutoFactory(robotState::getEstimatedPose, // A function that returns the current robot pose
        robotState::setEstimatedPose, // A function that resets the current robot pose to the provided Pose2d
        null, // The drive subsystem trajectory follower 
        true, // If alliance flipping should be enabled 
        swerve, // The drive subsystem
        null); 
    }


    public AutoRoutine exampleAutoRoutine() {
        AutoRoutine routine = factory.newRoutine("exampleRoutine"); //Name the routine
        AutoTrajectory driveSomewhere = routine.trajectory("NewPath"); //Load the trajectory

        routine.active().onTrue(
            Commands.sequence(
                driveSomewhere.resetOdometry(),
                driveSomewhere.cmd()
            )
        );

        return routine;
    }

    

}
