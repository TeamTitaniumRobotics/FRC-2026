package org.teamtitanium.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.PhotonCamera;

public class VisionConstants {
  public static final PhotonCamera camera = new PhotonCamera("LumaP1");

  public static final Transform3d robotToCamera =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0),
          new Rotation3d()); // TODO: Update this with the actual position of the camera on the
  // robot

  // No idea if all of these are necessary, but these are some of the constants from last year's
  // code that are related to vision

  public static final double maxAmbiguity = 0.3;
  public static final double maxZError = 0.75;

  public static final double linearStdDevBaseline = 0.02;
  public static final double angularStdDevBaseline = 0.06;

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
}
