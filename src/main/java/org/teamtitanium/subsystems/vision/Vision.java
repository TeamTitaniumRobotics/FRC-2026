package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.teamtitanium.RobotState;
import org.teamtitanium.utils.VirtualSubsystem;

public class Vision extends VirtualSubsystem {
  private final VisionIO io;
  private final RobotState robotState;

  public Vision(VisionIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
  }

  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> estimate = io.getEstimatedGlobalPose();
    estimate.ifPresent(
        est ->
            robotState.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, getVisionStdDevs()));
  }

  private Matrix<N3, N1> getVisionStdDevs() {
    return VecBuilder.fill(0.7, 0.7, Math.toRadians(30.0));
  }
}
