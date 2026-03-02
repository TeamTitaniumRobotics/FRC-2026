package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.teamtitanium.utils.FieldConstants;

public class VisionIOSim extends VisionIOPhoton {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final PhotonCameraSim cameraSim;

  public VisionIOSim(
      String cameraName, Transform3d robotToCamera, Supplier<Pose2d> robotPoseSupplier) {
    super(cameraName, robotToCamera);
    this.robotPoseSupplier = robotPoseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.defaultAprilTagType.getLayout());
    }

    var cameraProperties = new SimCameraProperties();

    cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
    cameraProperties.setCalibError(0.45, 0.1);
    cameraProperties.setFPS(30);
    cameraProperties.setAvgLatencyMs(30);
    cameraProperties.setLatencyStdDevMs(10);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(robotPoseSupplier.get());
    super.updateInputs(inputs);
  }
}
