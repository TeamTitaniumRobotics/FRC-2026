package org.teamtitanium.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public enum Points {
  // Bump Points
  RBA(new Point(new Pose2d(3.125, 2.5, Rotation2d.fromDegrees(-15.0)))),
  RBM(new Point(new Pose2d(5.65, 2.5, Rotation2d.fromDegrees(-15.0)))),
  // Middle Points
  RFM(new Point(new Pose2d(7.940, 3.3, Rotation2d.kZero))),
  // Trench Points
  RTS(new Point(new Pose2d(4.325, 0.65, Rotation2d.kZero)));

  @Getter private final Point point;

  Points(Point point) {
    this.point = point;
  }

  public void visualize() {
    Logger.recordOutput("Autos/Points/" + name(), point.getPose());
  }
}
