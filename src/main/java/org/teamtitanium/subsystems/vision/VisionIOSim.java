package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.teamtitanium.utils.FieldConstants;

public class VisionIOSim implements VisionIO {
  private final PhotonCamera camera = VisionConstants.camera;
  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          FieldConstants.defaultAprilTagType.getLayout(), VisionConstants.robotToCamera);
  private final VisionSystemSim visionSim = new VisionSystemSim("Vision");
  private final PhotonCameraSim cameraSim;

  public VisionIOSim() {
    var cameraProps = SimCameraProperties.PERFECT_90DEG();
    cameraProps.setFPS(20);
    cameraProps.setAvgLatencyMs(50);
    cameraProps.setLatencyStdDevMs(10);

    cameraSim =
        new PhotonCameraSim(camera, cameraProps, FieldConstants.defaultAprilTagType.getLayout());
    cameraSim.enableDrawWireframe(false);
    cameraSim.enableRawStream(false);
    cameraSim.enableProcessedStream(false);

    visionSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());
    visionSim.addCamera(cameraSim, VisionConstants.robotToCamera);
    visionSim.resetRobotPose(Pose2d.kZero);
  }

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

  // @Override
  // public void simulationPeriodic(Pose2d robotPose) {
  //   visionSim.update(robotPose);
  //   poseEstimator.addHeadingData(Timer.getTimestamp(), robotPose.getRotation());
  // }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    var latest = camera.getLatestResult();
    inputs.targetCount = latest.getTargets().size();

    inputs.timestampSeconds = latest.getTimestampSeconds();

    // Safely handle absent pose estimate to avoid NoSuchElementException
    Optional<EstimatedRobotPose> optEstimate = getEstimatedGlobalPose();
    if (optEstimate.isPresent()) {
      inputs.estimatedGlobalPose = optEstimate.get().estimatedPose;
    } else {
      // No estimate available — clear or leave null (depends on VisionIOInputs expectations)
      inputs.estimatedGlobalPose = null;
    }
  }
}
