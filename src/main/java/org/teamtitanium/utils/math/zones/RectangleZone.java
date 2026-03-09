package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.teamtitanium.utils.AllianceFlipUtil;

/**
 * A rectangular {@link IZone} backed by a WPILib {@link Rectangle2d}.
 *
 * <p>Alliance flipping is applied automatically at query time using {@link AllianceFlipUtil}, so
 * all constants can be defined from the blue-alliance perspective. WPILib's {@link Rectangle2d}
 * supports optional rotation via its {@link Pose2d} center constructor.
 *
 * <p>Example:
 *
 * <pre>{@code
 * IZone allianceZone = new RectangleZone(FieldConstants.Zones.allianceZone);
 * allianceZone.containsRobot(robotState::getEstimatedPose).onTrue(shooter.aim());
 * }</pre>
 */
public class RectangleZone implements IZone {

  private final Rectangle2d rectangle;

  /**
   * Creates a zone from an existing {@link Rectangle2d} defined in blue-alliance coordinates.
   *
   * @param rectangle the rectangle in blue-alliance field coordinates
   */
  public RectangleZone(Rectangle2d rectangle) {
    this.rectangle = rectangle;
  }

  /**
   * Creates an axis-aligned zone from two diagonally opposite corners (blue-alliance coordinates).
   *
   * @param corner1 one corner of the rectangle
   * @param corner2 the diagonally opposite corner
   */
  public RectangleZone(Translation2d corner1, Translation2d corner2) {
    this(new Rectangle2d(corner1, corner2));
  }

  /**
   * Creates an axis-aligned zone from a center pose and dimensions (blue-alliance coordinates).
   *
   * @param center center pose of the rectangle (translation + rotation)
   * @param xWidth full width along the rectangle's local X axis (meters)
   * @param yWidth full width along the rectangle's local Y axis (meters)
   */
  public RectangleZone(Pose2d center, double xWidth, double yWidth) {
    this(new Rectangle2d(center, xWidth, yWidth));
  }

  /**
   * Creates a rotated zone from a center position, dimensions, and rotation.
   *
   * @param center center translation of the rectangle (blue-alliance coordinates)
   * @param xWidth full width along the rotated X axis (meters)
   * @param yWidth full width along the rotated Y axis (meters)
   * @param rotation rotation of the rectangle on the field
   */
  public RectangleZone(Translation2d center, double xWidth, double yWidth, Rotation2d rotation) {
    this(new Rectangle2d(new Pose2d(center, rotation), xWidth, yWidth));
  }

  /**
   * {@inheritDoc}
   *
   * <p>The rectangle is alliance-flipped before testing so blue-alliance constants work on both
   * sides of the field.
   */
  @Override
  public boolean containsPoint(Translation2d point) {
    return AllianceFlipUtil.apply(rectangle).contains(point);
  }

  @Override
  public Trigger contains(Supplier<Translation2d> translationSupplier) {
    return new Trigger(() -> containsPoint(translationSupplier.get()));
  }

  /**
   * Returns the underlying {@link Rectangle2d} in blue-alliance coordinates (before flipping).
   *
   * @return the rectangle
   */
  public Rectangle2d getRectangle() {
    return rectangle;
  }
}
