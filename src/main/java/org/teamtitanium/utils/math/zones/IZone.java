package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

public interface IZone {
  /**
   * Returns a {@link Trigger} that is true when the given point (e.g. robot center) is inside this
   * zone.
   */
  public Trigger contains(Supplier<Translation2d> translationSupplier);

  /**
   * Returns a {@link Trigger} that is true if the given point will be inside this zone after {@code
   * dt} seconds, given the current field-relative chassis speeds.
   *
   * <p>The point is projected forward by applying the linear velocity offset {@code (vx·dt, vy·dt)}
   * and then tested with {@link #containsPoint}.
   */
  public default Trigger willContain(
      Supplier<Translation2d> translationSupplier, Supplier<ChassisSpeeds> fieldSpeeds, double dt) {
    return new Trigger(() -> willContainPoint(translationSupplier.get(), fieldSpeeds, dt));
  }

  /**
   * Returns a {@link Trigger} that is true if the given point is currently inside this zone OR will
   * be inside within {@code dt} seconds.
   */
  public default Trigger doesOrWillContain(
      Supplier<Translation2d> translationSupplier, Supplier<ChassisSpeeds> fieldSpeeds, double dt) {
    return contains(translationSupplier).or(willContain(translationSupplier, fieldSpeeds, dt));
  }

  /**
   * Returns a {@link Trigger} that is true when <em>any corner of the robot bumper</em> is inside
   * this zone. Uses {@link RobotFootprint} to derive the four bumper corners from the pose.
   */
  public default Trigger containsRobot(Supplier<Pose2d> poseSupplier, boolean fullyContains) {
    return new Trigger(
        () -> {
          if (fullyContains) {
            Translation2d[] corners = RobotFootprint.getBumperCorners(poseSupplier.get());
            for (Translation2d corner : corners) {
              if (!containsPoint(corner)) return false;
            }
            return true;
          }
          Translation2d[] corners = RobotFootprint.getBumperCorners(poseSupplier.get());
          for (Translation2d corner : corners) {
            if (containsPoint(corner)) return true;
          }
          return false;
        });
  }

  /**
   * Returns a {@link Trigger} that is true if the robot bumper will be inside this zone after
   * {@code dt} seconds, given the current field-relative chassis speeds.
   *
   * <p>Each bumper corner is projected forward by {@code (vx·dt, vy·dt)} and tested with {@link
   * #containsPoint}. The {@code fullyContains} flag mirrors the behaviour of {@link
   * #containsRobot}: when {@code true}, ALL corners must be inside; when {@code false}, ANY corner
   * is sufficient.
   */
  public default Trigger willContainRobot(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeeds,
      double dt,
      boolean fullyContains) {
    return new Trigger(
        () -> {
          ChassisSpeeds speeds = fieldSpeeds.get();
          Translation2d offset =
              new Translation2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt);
          Translation2d[] corners = RobotFootprint.getBumperCorners(poseSupplier.get());
          if (fullyContains) {
            for (Translation2d corner : corners) {
              if (!containsPoint(corner.plus(offset))) return false;
            }
            return true;
          }
          for (Translation2d corner : corners) {
            if (containsPoint(corner.plus(offset))) return true;
          }
          return false;
        });
  }

  /**
   * Low-level point-in-zone test used internally and by {@link #containsRobot}. Does NOT perform
   * alliance flipping — that is handled by each concrete implementation.
   */
  public boolean containsPoint(Translation2d point);

  public default boolean willContainPoint(
      Translation2d point, Supplier<ChassisSpeeds> fieldSpeeds, double dt) {
    ChassisSpeeds speeds = fieldSpeeds.get();
    Translation2d projected =
        point.plus(new Translation2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt));
    return containsPoint(projected);
  }

  // ── Set operations ────────────────────────────────────────────────────────

  /** Returns a new zone representing the union (A OR B) of this zone and {@code other}. */
  public default IZone union(IZone other) {
    return new CompositeZone(this, other, CompositeZone.Operation.UNION);
  }

  /** Returns a new zone representing the intersection (A AND B) of this zone and {@code other}. */
  public default IZone intersection(IZone other) {
    return new CompositeZone(this, other, CompositeZone.Operation.INTERSECTION);
  }

  /**
   * Returns a new zone representing the set difference (A AND NOT B): points in this zone but NOT
   * in {@code other}.
   */
  public default IZone difference(IZone other) {
    return new CompositeZone(this, other, CompositeZone.Operation.DIFFERENCE);
  }

  /** Returns a new zone representing the complement (NOT A): everywhere except this zone. */
  public default IZone complement() {
    return new CompositeZone(this, null, CompositeZone.Operation.COMPLEMENT);
  }

  /** Visualizes this zone with the given name. */
  public void visualize(String name);
}
