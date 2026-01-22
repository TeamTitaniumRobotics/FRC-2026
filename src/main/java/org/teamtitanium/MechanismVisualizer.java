package org.teamtitanium;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class MechanismVisualizer {
  private static MechanismVisualizer instance;

  private MechanismVisualizer() {
    // Private constructor to prevent instantiation
  }

  public static MechanismVisualizer getInstance() {
    if (instance == null) {
      instance = new MechanismVisualizer();
    }
    return instance;
  }

  @Getter @Setter private Rotation2d turretAngle = Rotation2d.kZero;
  @Getter @Setter private Rotation2d hoodAngle = Rotation2d.kZero;

  public void log(String key) {
    var turretPose =
        new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(
                new Transform3d(
                    new Translation3d(-0.5, 0.0, 0.35),
                    new Rotation3d(0.0, 0.0, turretAngle.getRadians())));
    var hoodPose =
        turretPose.transformBy(
            new Transform3d(
                new Translation3d(0.5, 0.0, 0.0),
                new Rotation3d(0.0, hoodAngle.getRadians(), 0.0)));
    Logger.recordOutput(key + "/Components", turretPose, hoodPose);

    var cameraPose =
        hoodPose.transformBy(
            new Transform3d(
                new Translation3d(-0.3, 0.0, 0.2),
                new Rotation3d(0.0, Units.degreesToRadians(-30), 0.0)));
    // .transformBy(turretPose.toTransform3d());
    // .transformBy(
    // new Transform3d(new Translation3d(0.3, 0.0, 0.2), new Rotation3d(0.0, 0.0,
    // 0.0)));
    Logger.recordOutput(key + "/Camera", cameraPose);
  }
}
