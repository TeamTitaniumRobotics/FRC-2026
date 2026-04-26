package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.teamtitanium.utils.FieldConstants;

public class VisionIOPhoton implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhoton(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;

    this.poseEstimator =
        new PhotonPoseEstimator(
            FieldConstants.defaultAprilTagType.getLayout(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();
    inputs.name = camera.getName();

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var result : camera.getAllUnreadResults()) {
      var visionEstimate =
          poseEstimator.update(
              result,
              Optional.empty(),
              Optional.empty(),
              Optional.of(new ConstrainedSolvepnpParams(false, 1.0)));

      if (visionEstimate.isPresent()) {
        var estimate = visionEstimate.get();

        double averageTagDistance = 0.0;
        double averagePoseAmbiguity = 0.0;
        for (var target : estimate.targetsUsed) {
          averageTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
          averagePoseAmbiguity += target.getPoseAmbiguity();
          tagIds.add(target.getFiducialId());
        }

        averageTagDistance /= estimate.targetsUsed.size();
        averagePoseAmbiguity /= estimate.targetsUsed.size();

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                estimate.estimatedPose,
                averagePoseAmbiguity,
                estimate.targetsUsed.size(),
                averageTagDistance,
                PoseObservationType.PHOTONVISION,
                estimate.strategy));
      }

      inputs.poseObservations = new PoseObservation[poseObservations.size()];
      for (int i = 0; i < poseObservations.size(); i++) {
        inputs.poseObservations[i] = poseObservations.get(i);
      }

      inputs.tagIds = new int[tagIds.size()];
      int i = 0;
      for (var tagId : tagIds) {
        inputs.tagIds[i++] = tagId;
      }
    }
  }
}
