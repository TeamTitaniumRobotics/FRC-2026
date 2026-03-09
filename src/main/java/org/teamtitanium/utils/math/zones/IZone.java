package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

public interface IZone {
  /**
   * Returns a {@link Trigger} that is true when the given point (e.g. robot center) is inside this
   * zone.
   */
  Trigger contains(Supplier<Translation2d> translationSupplier);

  /**
   * Returns a {@link Trigger} that is true when <em>any corner of the robot bumper</em> is inside
   * this zone. Uses {@link RobotFootprint} to derive the four bumper corners from the pose.
   */
  default Trigger containsRobot(Supplier<Pose2d> poseSupplier) {
    return new Trigger(
        () -> {
          Translation2d[] corners = RobotFootprint.getBumperCorners(poseSupplier.get());
          for (Translation2d corner : corners) {
            if (containsPoint(corner)) return true;
          }
          return false;
        });
  }

  /**
   * Low-level point-in-zone test used internally and by {@link #containsRobot}. Does NOT perform
   * alliance flipping — that is handled by each concrete implementation.
   */
  boolean containsPoint(Translation2d point);

  // ── Set operations ────────────────────────────────────────────────────────

  /** Returns a new zone representing the union (A OR B) of this zone and {@code other}. */
  default IZone union(IZone other) {
    return new CompositeZone(this, other, CompositeZone.Operation.UNION);
  }

  /** Returns a new zone representing the intersection (A AND B) of this zone and {@code other}. */
  default IZone intersection(IZone other) {
    return new CompositeZone(this, other, CompositeZone.Operation.INTERSECTION);
  }

  /**
   * Returns a new zone representing the set difference (A AND NOT B): points in this zone but NOT
   * in {@code other}.
   */
  default IZone difference(IZone other) {
    return new CompositeZone(this, other, CompositeZone.Operation.DIFFERENCE);
  }

  /** Returns a new zone representing the complement (NOT A): everywhere except this zone. */
  default IZone complement() {
    return new CompositeZone(this, null, CompositeZone.Operation.COMPLEMENT);
  }
}
