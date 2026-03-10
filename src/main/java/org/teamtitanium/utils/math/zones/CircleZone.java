package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.AllianceFlipUtil;

/**
 * A circular {@link IZone} defined by a center and radius.
 *
 * <p>Alliance flipping is applied to the center at query time so all coordinates can be defined
 * from the blue-alliance perspective.
 *
 * <p>Example — trigger when the robot (bumpers included) is within 2 m of the hub:
 *
 * <pre>{@code
 * IZone hubProximity = new CircleZone(FieldConstants.Hub.center, 2.0);
 * hubProximity.containsRobot(robotState::getEstimatedPose).onTrue(shooter.aim());
 * }</pre>
 */
public class CircleZone implements IZone {

  private final Translation2d center;
  private final double radiusMeters;

  /**
   * Creates a circular zone (blue-alliance coordinates).
   *
   * @param center center of the circle in blue-alliance field coordinates
   * @param radiusMeters radius of the circle in meters
   */
  public CircleZone(Translation2d center, double radiusMeters) {
    this.center = center;
    this.radiusMeters = radiusMeters;
  }

  /**
   * Creates a circular zone with a radius specified in inches (blue-alliance coordinates).
   *
   * @param center center of the circle in blue-alliance field coordinates
   * @param radiusInches radius of the circle in inches
   * @return a new {@link CircleZone}
   */
  public static CircleZone ofInches(Translation2d center, double radiusInches) {
    return new CircleZone(center, Units.inchesToMeters(radiusInches));
  }

  /**
   * {@inheritDoc}
   *
   * <p>The center is alliance-flipped before testing so blue-alliance constants work on both sides
   * of the field.
   */
  @Override
  public boolean containsPoint(Translation2d point) {
    Translation2d flippedCenter = AllianceFlipUtil.apply(center);
    return point.getDistance(flippedCenter) <= radiusMeters;
  }

  @Override
  public Trigger contains(Supplier<Translation2d> translationSupplier) {
    return new Trigger(() -> containsPoint(translationSupplier.get()));
  }

  /** Returns the blue-alliance center of this zone (before alliance flipping). */
  public Translation2d getCenter() {
    return center;
  }

  /** Returns the radius of this zone in meters. */
  public double getRadiusMeters() {
    return radiusMeters;
  }

  /**
   * Visualizes this zone as a set of points along the circumference of the circle. The center is
   * not included since it may be alliance-flipped.
   *
   * <p>The points are recorded under the key "{prefix}Points" as an array of {@link Translation2d}.
   *
   * @param prefix the prefix for the logged keys (e.g. "IntakeZone/") to log the points under
   */
  @Override
  public void visualize(String prefix) {
    Translation2d[] points = new Translation2d[16];
    for (int i = 0; i < points.length; i++) {
      double angle = 2 * Math.PI * i / points.length;
      points[i] =
          center.plus(
              new Translation2d(radiusMeters * Math.cos(angle), radiusMeters * Math.sin(angle)));
    }
    Logger.recordOutput(prefix + "Points", points);
  }
}
