package org.teamtitanium.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class VisionIOSim implements VisionIO {
  @Override
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return Optional.empty();
  }
}
