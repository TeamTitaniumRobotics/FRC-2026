package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final String forwardCameraName = "front";
  public static final String leftCameraName = "left";

  public static final Transform3d forwardCameraPose =
      new Transform3d(
          new Translation3d(-0.012729, 0.03125, 0.54657),
          new Rotation3d(0.0, Units.degreesToRadians(-6.0), Units.degreesToRadians(-20.0)));
  public static final Transform3d leftCameraPose =
      new Transform3d(
          new Translation3d(-0.011051, 0.083184, 0.545662),
          new Rotation3d(0.0, Units.degreesToRadians(-6.0), Units.degreesToRadians(50.0)));

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  public static final double linearStdDevBaseline = 0.02;
  public static final double angularStdDevBaseline = 0.06;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> DISABLED_STD_DEVS = VecBuilder.fill(0.05, 0.05, 0.1);
}
