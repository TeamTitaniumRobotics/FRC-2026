package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public boolean connected = false;
    public String name = "";

    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  public record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type,
      PoseStrategy poseStrategy) {}

  public enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  default Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return Optional.empty();
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void updateHeading(double timestamp, Rotation2d heading) {}
}
