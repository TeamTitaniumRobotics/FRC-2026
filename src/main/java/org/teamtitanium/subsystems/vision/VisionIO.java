package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
  Optional<EstimatedRobotPose> getEstimatedGlobalPose();

  default void simulationPeriodic(Pose2d robotPose) {}
}
