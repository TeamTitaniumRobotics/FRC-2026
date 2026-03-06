package org.teamtitanium.autos;

import choreo.auto.AutoFactory;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.AllianceFlipUtil;

public class AutoRoutines {
  private final Swerve swerve;
  private final AutoFactory factory;
  private final RobotState robotState = RobotState.getInstance();

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
}
