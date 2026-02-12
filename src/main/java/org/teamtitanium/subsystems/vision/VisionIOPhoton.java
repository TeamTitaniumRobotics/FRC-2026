package org.teamtitanium.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.cameraName);
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          VisionConstants.tagLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.robotToCamera);

  @Override
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
    if (estimate.isEmpty()) {
      estimate = poseEstimator.estimateLowestAmbiguityPose(result);
    }
    return estimate;
  }
}
