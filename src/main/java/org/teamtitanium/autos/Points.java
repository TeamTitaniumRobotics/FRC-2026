package org.teamtitanium.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.AllianceFlipUtil;

public enum Points {
  // Bump Points
  RBA(new Point(new Pose2d(3.125, 2.5, Rotation2d.fromDegrees(-15.0)), Point.PathType.Waypoint)),
  RBM(new Point(new Pose2d(5.65, 2.5, Rotation2d.fromDegrees(-15.0)), Point.PathType.Waypoint)),
  // Middle Points
  RFM(new Point(new Pose2d(7.940, 3.3, Rotation2d.fromDegrees(90.0)), Point.PathType.Waypoint)),
  // Trench Points
  RTS(new Point(new Pose2d(4.325, 0.65, Rotation2d.kZero), Point.PathType.Waypoint)),
  RTE(new Point(new Pose2d(5.5, 0.65, Rotation2d.kZero), Point.PathType.Waypoint));

  @Getter private final Point point;

  Points(Point point) {
    this.point = point;
  }

  public void visualize() {
    Logger.recordOutput("Autos/Points/" + name(), point.getPose());
  }

  public record Point(Pose2d pose, PathType type) {
    public Pose2d getPose() {
      return AllianceFlipUtil.apply(pose);
    }

    public PathElement toPathElement(double tRatio) {
      return switch (type) {
        case Waypoint -> toWaypoint();
        case TranslationTarget -> toTranslationTarget();
        case RotationTarget -> toRotationTarget(tRatio);
      };
    }

    public Waypoint toWaypoint() {
      return new Waypoint(getPose());
    }

    public TranslationTarget toTranslationTarget() {
      return new TranslationTarget(getPose().getTranslation());
    }

    public RotationTarget toRotationTarget(double tRatio) {
      return new RotationTarget(getPose().getRotation(), tRatio);
    }

    public enum PathType {
      Waypoint,
      TranslationTarget,
      RotationTarget
    }
  }
}
