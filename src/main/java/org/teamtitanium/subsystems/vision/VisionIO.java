package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {

  @AutoLog
  public class VisionIOInputs {
    public boolean connected = false; // Whether the camera is connected

    public int targetCount = 0; // Number of targets currently detected

    public double timestampSeconds = 0; // Timestamp (in seconds) of the last update

    public Pose3d estimatedGlobalPose =
        new Pose3d(); // Latest estimated pose of the robot, pose3d because
    // EstimatedRobotPose isn't supported by AutoLog
  }

  default Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return Optional.empty();
  }

  default void updateInputs(VisionIOInputs inputs) {}

  default void simulationPeriodic(Pose2d robotPose) {}
}
