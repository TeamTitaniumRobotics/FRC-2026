package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

/**
 * A zone built by combining two existing {@link IZone}s via a set operation.
 *
 * <p>Implements union ($A \cup B$), intersection ($A \cap B$), set difference ($A \setminus B$),
 * and complement ($\overline{A}$). Because {@link CompositeZone} itself implements {@link IZone},
 * operations can be chained arbitrarily:
 *
 * <pre>{@code
 * IZone safe = allianceZone.difference(trenchZone).difference(bumpZone);
 * }</pre>
 */
public class CompositeZone implements IZone {

  public enum Operation {
    UNION,
    INTERSECTION,
    DIFFERENCE,
    COMPLEMENT
  }

  private final IZone left;
  private final IZone right; // null for COMPLEMENT
  private final Operation operation;

  /**
   * @param left the primary zone
   * @param right the secondary zone (may be {@code null} for {@link Operation#COMPLEMENT})
   * @param operation the set operation to apply
   */
  public CompositeZone(IZone left, IZone right, Operation operation) {
    this.left = left;
    this.right = right;
    this.operation = operation;
  }

  @Override
  public boolean containsPoint(Translation2d point) {
    return switch (operation) {
      case UNION -> left.containsPoint(point) || right.containsPoint(point);
      case INTERSECTION -> left.containsPoint(point) && right.containsPoint(point);
      case DIFFERENCE -> left.containsPoint(point) && !right.containsPoint(point);
      case COMPLEMENT -> !left.containsPoint(point);
    };
  }

  @Override
  public Trigger contains(Supplier<Translation2d> translationSupplier) {
    return new Trigger(() -> containsPoint(translationSupplier.get()));
  }

  @Override
  public void visualize(String prefix) {
    left.visualize(prefix + "Left/");
    if (right != null) {
      right.visualize(prefix + "Right/");
    }
  }
}
