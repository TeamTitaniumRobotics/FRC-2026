package org.teamtitanium.autos;

import java.util.List;
import org.teamtitanium.autos.AutoRoutines.PathAction;

public enum Paths {
  RTS_RFM(List.of(Points.RTS, Points.RFM), AutoRoutines.PathAction.INTAKE);

  private final Path path;

  Paths(List<Points> points, AutoRoutines.PathAction action) {
    this.path = new Path(points, action);
  }

  public Path getPath() {
    return path;
  }

  public record Path(List<Points> points, PathAction action, boolean intakeDeployed) {
    public Path(List<Points> points, PathAction action) {
      this(points, action, action == PathAction.INTAKE);
    }

    public frc.robot.lib.BLine.Path getPath() {
      final var path =
          new frc.robot.lib.BLine.Path(
              points.stream().map(p -> p.getPoint().toPathElement(0.5)).toList());
      return path;
    }
  }
}
