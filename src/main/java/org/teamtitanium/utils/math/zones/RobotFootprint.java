package org.teamtitanium.utils.math.zones;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.teamtitanium.utils.Constants;

/**
 * Utility class that derives the four bumper corners of the robot from a {@link Pose2d}.
 *
 * <p>Dimensions are sourced from {@link Constants#BUMPER_HALF_LENGTH} and {@link
 * Constants#BUMPER_HALF_WIDTH}, which are computed from the actual swerve module locations in
 * {@link org.teamtitanium.utils.TunerConstants} plus the bumper extension.
 */
public class RobotFootprint {

  private RobotFootprint() {}

  /**
   * Returns the four bumper corners of the robot in field coordinates, derived from {@code pose}.
   *
   * <p>Corner order: front-left, front-right, back-right, back-left (clockwise from front-left).
   *
   * @param pose the current robot pose in field coordinates
   * @return array of four {@link Translation2d} bumper corners
   */
  public static Translation2d[] getBumperCorners(Pose2d pose) {
    double dx = Constants.BUMPER_HALF_LENGTH.in(Meters);
    double dy = Constants.BUMPER_HALF_WIDTH.in(Meters);
    double cos = pose.getRotation().getCos();
    double sin = pose.getRotation().getSin();

    return new Translation2d[] {
      rotateAndTranslate(pose, dx, dy, cos, sin), // front-left
      rotateAndTranslate(pose, dx, -dy, cos, sin), // front-right
      rotateAndTranslate(pose, -dx, -dy, cos, sin), // back-right
      rotateAndTranslate(pose, -dx, dy, cos, sin) // back-left
    };
  }

  private static Translation2d rotateAndTranslate(
      Pose2d pose, double offsetX, double offsetY, double cos, double sin) {
    return new Translation2d(
        pose.getX() + offsetX * cos - offsetY * sin, pose.getY() + offsetX * sin + offsetY * cos);
  }
}
