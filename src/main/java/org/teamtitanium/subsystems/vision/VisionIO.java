package org.teamtitanium.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
  Optional<EstimatedRobotPose> getEstimatedGlobalPose();
}
