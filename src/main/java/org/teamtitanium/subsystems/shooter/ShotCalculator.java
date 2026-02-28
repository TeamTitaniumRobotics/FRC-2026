package org.teamtitanium.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import org.teamtitanium.RobotState;
import org.teamtitanium.utils.AllianceFlipUtil;
import org.teamtitanium.utils.FieldConstants;
import org.teamtitanium.utils.LoggedTunableNumber;

public class ShotCalculator {
  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  public static final LoggedTunableNumber maxFlywheelIdleRPM =
      new LoggedTunableNumber("ShotCalculator/MaxFlywheelIdleRPM", 30.0);

  private ShotParameters latestParameters = new ShotParameters(false, 0, 0, 0, 0, 0, 0, false);

  private static final InterpolatingTreeMap<Double, ShotData> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);
  private static final InterpolatingTreeMap<Double, ShotData> passingMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

  static {
    shotMap.put(3.31, new ShotData(3240, 8));
    shotMap.put(3.05, new ShotData(3120, 10));
    shotMap.put(2.75, new ShotData(3000, 9));
    shotMap.put(2.50, new ShotData(2880, 8));
    shotMap.put(1.50, new ShotData(2580, 4));
  }

  public ShotParameters getParameters() {
    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();

    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    double targetDistance = target.getDistance(estimatedPose.getTranslation());

    // double hoodAngleRots = shotMap.get(targetDistance).flywheelRPM();
    // double flywheelRPM = shotMap.get(targetDistance).flywheelRPM();
    // double flywheelIdleRPM = MathUtil.clamp(0.0, 0.0, maxFlywheelIdleRPM.get());

    latestParameters =
        new ShotParameters(true, 0.0, 0.0, 0.0, targetDistance, targetDistance, 0.0, false);

    return latestParameters;
  }

  public void resetShotParameters() {
    latestParameters = null;
  }

  public record ShotParameters(
      boolean isValid,
      double hoodAngleDegs,
      double flywheelRPM,
      double flywheelIdleRPM,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  public record ShotData(double flywheelRPM, double hoodAngleDegs, double tof) {
    public ShotData(double flywheelRPM, double hoodAngleDegs) {
      this(flywheelRPM, hoodAngleDegs, 0.0);
    }

    public static ShotData interpolate(ShotData start, ShotData end, double t) {
      return new ShotData(
          MathUtil.interpolate(start.flywheelRPM, end.flywheelRPM, t),
          MathUtil.interpolate(start.hoodAngleDegs, end.hoodAngleDegs, t),
          MathUtil.interpolate(start.tof, end.tof, t));
    }
  }
}
