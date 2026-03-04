package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.RobotState.VisionObservation;
import org.teamtitanium.utils.FieldConstants;
import org.teamtitanium.utils.virtualsubsystem.VirtualSubsystem;

public class Vision extends VirtualSubsystem {
  private final VisionIO[] visionIOs;
  private final VisionIOInputsAutoLogged[] visionInputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionIO... visionIOs) {
    this.visionIOs = visionIOs;

    this.visionInputs =
        Arrays.stream(visionIOs)
            .map(visionIO -> new VisionIOInputsAutoLogged())
            .toArray(VisionIOInputsAutoLogged[]::new);
    this.disconnectedAlerts =
        Arrays.stream(visionIOs)
            .map(visionIO -> new Alert("Vision camera is disconnected.", Alert.AlertType.kError))
            .toArray(Alert[]::new);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(visionInputs[i]);
      Logger.processInputs("Vision/Camera" + i, visionInputs[i]);
    }

    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop through cameras
    for (int cameraIndex = 0; cameraIndex < visionIOs.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!visionInputs[cameraIndex].connected);

      // Initialize pose logging
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : visionInputs[cameraIndex].tagIds) {
        var tagPose = FieldConstants.defaultAprilTagType.getLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop through pose observations
      for (var observation : visionInputs[cameraIndex].poseObservations) {
        // Pose rejection criteria
        boolean rejectPose =
            observation.tagCount() == 0
                || (observation.tagCount() == 1
                    && observation.ambiguity() > VisionConstants.maxAmbiguity)
                || Math.abs(observation.pose().getZ()) > VisionConstants.maxZError
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FieldConstants.fieldLength
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FieldConstants.fieldWidth;

        // Add robot poses to logs
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        if (rejectPose) {
          continue;
        }

        var estStdDevs =
            observation.tagCount() > 1
                ? VisionConstants.MULTI_TAG_STD_DEVS
                : VisionConstants.SINGLE_TAG_STD_DEVS;

        if (observation.tagCount() == 1 && observation.averageTagDistance() > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs =
              estStdDevs.times(
                  1
                      + (observation.averageTagDistance()
                          * observation.averageTagDistance()
                          / (observation.tagCount() * 15.0)));
        }

        if (DriverStation.isDisabled()) {
          estStdDevs = VisionConstants.DISABLED_STD_DEVS;
        }

        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionObservation(observation.pose(), observation.timestamp(), estStdDevs));
      }

      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }
}
