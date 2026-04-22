package org.teamtitanium.autos;

import edu.wpi.first.math.geometry.Pose2d;
import org.teamtitanium.utils.AllianceFlipUtil;

public record Point(Pose2d pose) {
  public Pose2d getPose() {
    return AllianceFlipUtil.apply(pose);
  }
}
