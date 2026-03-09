package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Arrays;
import java.util.function.Supplier;
import org.teamtitanium.utils.AllianceFlipUtil;

/**
 * A polygonal {@link IZone} defined by an ordered list of vertices.
 *
 * <p>Point containment is tested using the <em>winding number</em> algorithm, which correctly
 * handles both convex and concave polygons. Alliance flipping is applied to every vertex at query
 * time so all constants can be defined in blue-alliance coordinates.
 *
 * <p>Vertices must be provided in order (clockwise or counter-clockwise). The polygon is
 * automatically closed — you do not need to repeat the first vertex at the end.
 *
 * <p>Example — L-shaped exclusion zone:
 *
 * <pre>{@code
 * IZone lShape = new PolygonZone(
 *     new Translation2d(1.0, 1.0),
 *     new Translation2d(3.0, 1.0),
 *     new Translation2d(3.0, 2.0),
 *     new Translation2d(2.0, 2.0),
 *     new Translation2d(2.0, 4.0),
 *     new Translation2d(1.0, 4.0));
 * lShape.containsRobot(robotState::getEstimatedPose).onTrue(hood.retract());
 * }</pre>
 */
public class PolygonZone implements IZone {

  private final Translation2d[] vertices; // blue-alliance coordinates, ordered

  /**
   * Creates a polygonal zone from an ordered array of vertices in blue-alliance field coordinates.
   *
   * @param vertices ordered polygon vertices (at least 3); do not repeat the first vertex
   * @throws IllegalArgumentException if fewer than 3 vertices are provided
   */
  public PolygonZone(Translation2d... vertices) {
    if (vertices.length < 3) {
      throw new IllegalArgumentException(
          "PolygonZone requires at least 3 vertices, got " + vertices.length);
    }
    this.vertices = Arrays.copyOf(vertices, vertices.length);
  }

  /**
   * {@inheritDoc}
   *
   * <p>All vertices are alliance-flipped before testing so blue-alliance constants work on both
   * sides of the field. Uses the winding-number algorithm — handles convex and concave polygons.
   */
  @Override
  public boolean containsPoint(Translation2d point) {
    Translation2d[] flipped = new Translation2d[vertices.length];
    for (int i = 0; i < vertices.length; i++) {
      flipped[i] = AllianceFlipUtil.apply(vertices[i]);
    }
    return windingNumber(point, flipped) != 0;
  }

  @Override
  public Trigger contains(Supplier<Translation2d> translationSupplier) {
    return new Trigger(() -> containsPoint(translationSupplier.get()));
  }

  /**
   * Returns a copy of the polygon's vertices in blue-alliance coordinates (before flipping).
   *
   * @return vertex array copy
   */
  public Translation2d[] getVertices() {
    return Arrays.copyOf(vertices, vertices.length);
  }

  // ── Winding Number Algorithm ─────────────────────────────────────────────
  // Reference: http://geomalgorithms.com/a03-_inclusion.html (W. Randolph Franklin)
  //
  // The winding number counts how many times the polygon winds around the test point.
  // A non-zero result means the point is inside the polygon.

  private static int windingNumber(Translation2d point, Translation2d[] poly) {
    int winding = 0;
    int n = poly.length;

    for (int i = 0; i < n; i++) {
      Translation2d a = poly[i];
      Translation2d b = poly[(i + 1) % n];

      if (a.getY() <= point.getY()) {
        // Upward crossing
        if (b.getY() > point.getY() && isLeft(a, b, point) > 0) {
          winding++;
        }
      } else {
        // Downward crossing
        if (b.getY() <= point.getY() && isLeft(a, b, point) < 0) {
          winding--;
        }
      }
    }
    return winding;
  }

  /**
   * Returns the signed area of the triangle formed by edge (a→b) and point p. Positive → p is left
   * of a→b; zero → collinear; negative → right of a→b.
   */
  private static double isLeft(Translation2d a, Translation2d b, Translation2d p) {
    return (b.getX() - a.getX()) * (p.getY() - a.getY())
        - (p.getX() - a.getX()) * (b.getY() - a.getY());
  }
}
