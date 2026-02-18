package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.teamtitanium.RobotState;
import org.teamtitanium.utils.VirtualSubsystem;

public class Vision extends VirtualSubsystem {
  private final VisionIO io;
  private final RobotState robotState;

  private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
  }

  public void simulationPeriodic(Pose2d robotPose) {
    io.simulationPeriodic(robotPose);
  }

  private Matrix<N3, N1> getVisionStdDevs() {
    return inputs.targetCount == 1
        ? VisionConstants.SINGLE_TAG_STD_DEVS
        : VisionConstants.MULTI_TAG_STD_DEVS;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Optional<EstimatedRobotPose> estimate = io.getEstimatedGlobalPose();
    estimate.ifPresent(
        est ->
            robotState.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, getVisionStdDevs()));
  }
}
