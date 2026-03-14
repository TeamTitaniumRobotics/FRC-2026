package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class CRTHelper {
  private static final double COMMON_RATIO = (85.0 / 10.0);
  private static final double RESOLUTION = 0.1;
  private static final int NUM_POINTS =
      (int) Math.floor((MAX_ANGLE.in(Degrees) - MIN_ANGLE.in(Degrees)) / RESOLUTION) + 1;
  private static int leastDistanceIndex = 0;

  private static final double[] MECHANISM_ANGLES = new double[NUM_POINTS];
  private static final double[] ARRAY_20T = generateEncoderValues(20);
  private static final double[] ARRAY_21T = generateEncoderValues(21);

  private static double[] generateEncoderValues(int teeth) {
    double[] values = new double[NUM_POINTS];
    double gearRatio = COMMON_RATIO * (50.0 / teeth);

    int i = 0;
    for (double angle = MIN_ANGLE.in(Degrees);
        angle < MAX_ANGLE.in(Degrees);
        angle += RESOLUTION, i++) {
      MECHANISM_ANGLES[i] = angle;
      values[i] = (((angle * gearRatio) % 360.0) + 360.0) % 360.0;
    }

    return values;
  }

  public static Rotation2d getAbsoluteAngle(double encoder20TValue, double encoder21TValue) {
    double leastDistance = Double.MAX_VALUE;
    int leastDistanceIndex = 0;

    for (int i = 0; i < NUM_POINTS; i++) {
      double diff20 = Math.abs(encoder20TValue - ARRAY_20T[i]);
      double diff21 = Math.abs(encoder21TValue - ARRAY_21T[i]);

      diff20 = Math.min(diff20, 360.0 - diff20);
      diff21 = Math.min(diff21, 360.0 - diff21);

      double distance20 = diff20 * diff20;
      double distance21 = diff21 * diff21;

      double distance = distance20 + distance21;

      if (distance < leastDistance) {
        leastDistance = distance;
        leastDistanceIndex = i;
      }
    }

    Logger.recordOutput("Turret/CRT/LeastDistance", leastDistance);
    Logger.recordOutput("Turret/CRT/LeastDistanceIndex", leastDistanceIndex);
    Logger.recordOutput("Turret/CRT/MechAngle", MECHANISM_ANGLES[leastDistanceIndex]);

    return Rotation2d.fromDegrees(MECHANISM_ANGLES[leastDistanceIndex]);
  }
}
