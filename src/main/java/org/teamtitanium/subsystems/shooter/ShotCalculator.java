package org.teamtitanium.subsystems.shooter;

import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.TURRET_TO_ROBOT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.utils.AllianceFlipUtil;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.FieldConstants;
import org.teamtitanium.utils.LoggedTunableNumber;
import org.teamtitanium.utils.math.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class ShotCalculator {
  private static ShotCalculator instance;

  public static ShotCalculator getInstance() {
    if (instance == null) {
      instance = new ShotCalculator();
    }
    return instance;
  }

  public static final LoggedTunableNumber maxFlywheelIdleRPM =
      new LoggedTunableNumber("ShotCalculator/MaxFlywheelIdleRPM", 0.0);

  @AutoLogOutput(key = "ShotCalculator/FlywheelOffset")
  private double flywheelOffset = 0.0;

  @AutoLogOutput(key = "ShotCalculator/HoodOffset")
  private double hoodOffset = 0.0;

  private ShotParameters latestParameters = null;

  private static final double phaseDelay = 0.03; // Turret response delay

  private static final InterpolatingTreeMap<Double, ShotData> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);
  private static final InterpolatingTreeMap<Double, ShotData> passingMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

  static {
    shotMap.put(1.48, new ShotData(1850, 0, 1.06));
    shotMap.put(1.75, new ShotData(1900, 0, 1.08));
    shotMap.put(2.00, new ShotData(1975, 0, 1.09));
    shotMap.put(2.25, new ShotData(2075, 0, 1.10));
    shotMap.put(2.50, new ShotData(2200, 0, 1.10));
    shotMap.put(2.75, new ShotData(2200, 0.75, 1.10));
    shotMap.put(3.00, new ShotData(2350, 3, 1.10));
    shotMap.put(3.25, new ShotData(2400, 4, 1.10));
    shotMap.put(3.50, new ShotData(2450, 5, 1.10));
    shotMap.put(3.75, new ShotData(2575, 7, 1.10));
    shotMap.put(4.00, new ShotData(2625, 8, 1.12));
    shotMap.put(4.25, new ShotData(2700, 9, 1.12));
    shotMap.put(4.50, new ShotData(2750, 10, 1.12));
    shotMap.put(4.75, new ShotData(2775, 11, 1.13));
    shotMap.put(5.00, new ShotData(2825, 13, 1.13));
    shotMap.put(5.25, new ShotData(2900, 14, 1.13));
    shotMap.put(5.50, new ShotData(2950, 15, 1.13));
    shotMap.put(5.89, new ShotData(3025, 15.5, 1.14));
    // shotMap.put(1.15, new ShotData(2500, 0));
    // shotMap.put(1.25, new ShotData(2525, 0));
    // shotMap.put(1.50, new ShotData(2550, 2.5));
    // shotMap.put(1.75, new ShotData(2600, 4));
    // shotMap.put(2.00, new ShotData(2650, 5));
    // shotMap.put(2.50, new ShotData(2750, 7, 1.1));
    // shotMap.put(2.75, new ShotData(2750, 10, 1.1));
    // shotMap.put(3.00, new ShotData(2850, 11, 1.1));
    // shotMap.put(3.25, new ShotData(3000, 12, 1.1));
    // shotMap.put(3.50, new ShotData(3075, 13, 1.1));
    // shotMap.put(3.75, new ShotData(3200, 14, 1.1));
    // shotMap.put(4.00, new ShotData(3400, 14.5, 1.12));
    // shotMap.put(4.25, new ShotData(3600, 14.5, 1.12));
    // shotMap.put(4.55, new ShotData(3800, 15, 1.12));
    // shotMap.put(4.75, new ShotData(4000, 16, 1.13));
    // shotMap.put(5.0, new ShotData(4100, 18, 1.13));
    // shotMap.put(5.25, new ShotData(4250, 19, 1.13));
    // shotMap.put(5.5, new ShotData(4450, 20, 1.13));

    passingMap.put(5.0, new ShotData(3350, 20.0, 1.13));
    // passingMap.put(6.0, new ShotData(3600, 22.5, 1.13));
    // passingMap.put(7.0, new ShotData(4000, 25.0, 1.14));
    // passingMap.put(8.0, new ShotData(4500, 27.5, 1.14));
    // passingMap.put(9.0, new ShotData(5250, 27.5, 1.15));
    // passingMap.put(10.0, new ShotData(5700, 27.5, 1.15));
  }

  public ShotParameters getParameters() {
    boolean passing = !RobotState.getInstance().inAllianceZone.getAsBoolean();

    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotVelocity = RobotState.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    if (passing) {
      if (AllianceFlipUtil.shouldFlip()) {
        target =
            RobotState.getInstance().getEstimatedPose().getY()
                    < FieldConstants.LinesHorizontal.center
                ? AllianceFlipUtil.apply(FieldConstants.PassingTargets.depotTarget)
                : AllianceFlipUtil.apply(FieldConstants.PassingTargets.outpostTarget);
      } else {
        target =
            RobotState.getInstance().getEstimatedPose().getY()
                    > FieldConstants.LinesHorizontal.center
                ? AllianceFlipUtil.apply(FieldConstants.PassingTargets.depotTarget)
                : AllianceFlipUtil.apply(FieldConstants.PassingTargets.outpostTarget);
      }
    }

    Pose2d turretPosition = estimatedPose.transformBy(TURRET_TO_ROBOT.toTransform2d());
    double targetToTurretDistance = target.getDistance(turretPosition.getTranslation());

    if (Constants.tuningMode) {
      Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
      Logger.recordOutput("ShotCalculator/TurretPose", turretPosition);
    }

    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    ChassisSpeeds turretVelocity =
        GeomUtil.transformVelocity(
            fieldVelocity,
            TURRET_TO_ROBOT.getTranslation().toTranslation2d(),
            RobotState.getInstance().getRotation());

    double tof =
        passing
            ? passingMap.get(targetToTurretDistance).tof()
            : shotMap.get(targetToTurretDistance).tof();
    Pose2d predictedTurretPose = turretPosition;
    double predictedDistance = targetToTurretDistance;

    for (int i = 0; i < 5; i++) {
      tof = shotMap.get(predictedDistance).tof();
      double offsetX = turretVelocity.vxMetersPerSecond * tof;
      double offsetY = turretVelocity.vyMetersPerSecond * tof;
      predictedTurretPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretPosition.getRotation());
      predictedDistance = target.getDistance(predictedTurretPose.getTranslation());
    }

    Pose2d predictedRobotPose =
        predictedTurretPose.transformBy(TURRET_TO_ROBOT.toTransform2d().inverse());

    Rotation2d turretAngle = getTurretAngle(predictedRobotPose, target);
    double hoodAngleRots =
        passing
            ? passingMap.get(predictedDistance).getHoodAngleRots()
            : shotMap.get(predictedDistance).getHoodAngleRots();
    double flywheelRPM =
        passing
            ? passingMap.get(predictedDistance).flywheelRPM() + flywheelOffset
            : shotMap.get(predictedDistance).flywheelRPM() + flywheelOffset;
    double flywheelIdleRPM = MathUtil.clamp(flywheelRPM, 0.0, maxFlywheelIdleRPM.get());

    boolean isValid =
        FieldConstants.Zones.NO_SHOOT_ZONE
            .contains(() -> turretPosition.getTranslation())
            .negate()
            .getAsBoolean();

    latestParameters =
        new ShotParameters(
            isValid,
            turretAngle.getRotations(),
            hoodAngleRots,
            flywheelRPM,
            flywheelIdleRPM,
            predictedDistance,
            targetToTurretDistance,
            tof,
            passing);

    if (Constants.tuningMode) {
      Logger.recordOutput("ShotCalculator/PredictedPose", predictedRobotPose);
      Logger.recordOutput("ShotCalculator/PredictedDistance", predictedDistance);
    }

    return latestParameters;
  }

  private static Rotation2d getTurretAngle(Pose2d robotPose, Translation2d target) {
    Pose2d turretPose = robotPose.transformBy(TURRET_TO_ROBOT.toTransform2d());

    Rotation2d turretToTarget = target.minus(turretPose.getTranslation()).getAngle();

    return turretToTarget.minus(robotPose.getRotation());
  }

  public void resetShotParameters() {
    latestParameters = null;
  }

  public void incrementFlywheelOffset(double incrementValue) {
    flywheelOffset += incrementValue;
  }

  public void setFlywheelOffset(double offset) {
    flywheelOffset = offset;
  }

  public void incrementHoodOffset(double incrementValue) {
    hoodOffset += incrementValue;
  }

  public void setHoodOffset(double offset) {
    hoodOffset = offset;
  }

  public record ShotParameters(
      boolean isValid,
      double turretAngleRots,
      double hoodAngleRots,
      double flywheelRPM,
      double flywheelIdleRPM,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  public record ShotData(double flywheelRPM, double hoodAngleDegs, double tof) {
    public ShotData(double flywheelRPM, double hoodAngleDegs) {
      this(flywheelRPM, hoodAngleDegs, 0.95);
    }

    public double getHoodAngleRots() {
      return Units.degreesToRotations(hoodAngleDegs());
    }

    public static ShotData interpolate(ShotData start, ShotData end, double t) {
      return new ShotData(
          MathUtil.interpolate(start.flywheelRPM, end.flywheelRPM, t),
          MathUtil.interpolate(start.hoodAngleDegs, end.hoodAngleDegs, t),
          MathUtil.interpolate(start.tof, end.tof, t));
    }
  }
}
