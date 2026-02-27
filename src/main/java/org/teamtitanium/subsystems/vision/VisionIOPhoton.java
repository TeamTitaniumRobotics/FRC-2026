package org.teamtitanium.subsystems.vision;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.teamtitanium.utils.FieldConstants;

public class VisionIOPhoton implements VisionIO {
  private final PhotonCamera camera = VisionConstants.camera; // Get camera from VisionConstants
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.defaultAprilTagType
              .getLayout(), // The layout of the AprilTags on the field, specified in FieldConstants
          VisionConstants
              .robotToCamera); // The transformation from the robot's center(?) to the camera,

  // specified in VisionConstants

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    inputs.targetCount = camera.getLatestResult().getTargets().size();

    inputs.timestampSeconds = camera.getLatestResult().getTimestampSeconds();

    inputs.estimatedGlobalPose = getEstimatedGlobalPose().get().estimatedPose;
  }

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
