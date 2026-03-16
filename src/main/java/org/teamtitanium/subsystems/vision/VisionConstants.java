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
  public static final String frontCameraName = "front";
  public static final String backCameraName = "back";
  public static final String leftCameraName = "left";
  public static final String frCameraName = "frontRight";

  public static final Transform3d frontCameraPose =
      new Transform3d(
          new Translation3d(-0.282414, 0.306387, 0.510389),
          new Rotation3d(0.0, Units.degreesToRadians(-22.0), Units.degreesToRadians(0.0)));
  public static final Transform3d backCameraPose =
      new Transform3d(
          new Translation3d(-0.304646, 0.300480, 0.325744),
          new Rotation3d(0.0, Units.degreesToRadians(-28.0), Units.degreesToRadians(247.22839)));
  public static final Transform3d leftCameraPose =
      new Transform3d(
          new Translation3d(-0.265486, 0.350036, 0.446487),
          new Rotation3d(0.0, Units.degreesToRadians(-22.0), Units.degreesToRadians(80.734836)));
  public static final Transform3d frCameraPose =
      new Transform3d(
          new Translation3d(-0.287570, -0.340613, 0.534458),
          new Rotation3d(0.0, Units.degreesToRadians(-22.0), Units.degreesToRadians(-71.511425)));

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  public static final double linearStdDevBaseline = 0.02;
  public static final double angularStdDevBaseline = 0.06;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  public static final Matrix<N3, N1> DISABLED_STD_DEVS = VecBuilder.fill(0.25, 0.25, 0.375);
}
