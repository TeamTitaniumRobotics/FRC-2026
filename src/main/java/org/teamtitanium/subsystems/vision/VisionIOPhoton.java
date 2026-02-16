package org.teamtitanium.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera =
      new PhotonCamera(
          VisionConstants
              .cameraName); // Create a new camera with the name specified in VisionConstants
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          VisionConstants
              .tagLayout, // The layout of the AprilTags on the field, specified in VisionConstants
          VisionConstants
              .robotToCamera); // The transformation from the robot's coordinate system to the

  // camera's coordinate system, specified in VisionConstants

  /*
   * This method gets the latest result from the camera and tries to estimate the robot's pose using the PhotonPoseEstimator.
   * If no targets are detected, it returns an empty Optional. If the coprocessor estimate is not available, it gets the estimate with the lowest ambiguity.
   */
  @Override
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }

    var result = results.get(results.size() - 1);
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
