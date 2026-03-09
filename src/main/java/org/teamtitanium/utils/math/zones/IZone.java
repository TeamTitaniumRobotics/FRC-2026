package org.teamtitanium.utils.math.zones;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;

public interface IZone {
  Trigger contains(Supplier<Pose2d> poseSupplier);

  IZone union(IZone other);

  IZone intersection(IZone other);

  IZone difference(IZone other);

  IZone complement();
}
