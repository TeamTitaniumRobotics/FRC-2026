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
    shotMap.put(1.75, new ShotData(1675, 0));
    shotMap.put(2.00, new ShotData(1750, 0));
    shotMap.put(2.25, new ShotData(1875, 0));
    shotMap.put(2.50, new ShotData(2025, 0));
    shotMap.put(2.75, new ShotData(2100, 0));
    shotMap.put(3.00, new ShotData(2275, 0));
    shotMap.put(3.25, new ShotData(2400, 1));
    shotMap.put(3.50, new ShotData(2500, 1));
    shotMap.put(3.75, new ShotData(2550, 3));
    shotMap.put(4.00, new ShotData(2700, 3));
    shotMap.put(4.25, new ShotData(2725, 4));
    shotMap.put(4.50, new ShotData(2750, 5));
    shotMap.put(4.75, new ShotData(2850, 5));
    shotMap.put(5.00, new ShotData(2900, 6));
    shotMap.put(5.25, new ShotData(2850, 8));
    shotMap.put(5.88, new ShotData(2950, 9));

    passingMap.put(4.00, new ShotData(2000, 15, 2.39, 2000));
    passingMap.put(5.00, new ShotData(2400, 17.5, 2.475, 2400));
    passingMap.put(6.00, new ShotData(2600, 20, 2.65, 2600));
    passingMap.put(7.00, new ShotData(2800, 22.5, 2.805, 2800));
    passingMap.put(8.00, new ShotData(3000, 25, 2.82, 3000));
    passingMap.put(9.00, new ShotData(3200, 27.5, 2.84, 3200));
    passingMap.put(9.80, new ShotData(3400, 30, 2.8675, 3400));
    passingMap.put(11.25, new ShotData(4400, 27, 3.26, 4400));
    passingMap.put(12.75, new ShotData(4775, 29, 1.9, 5850));
    passingMap.put(14.50, new ShotData(5525, 31, 1.635, 7350));
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

    InterpolatingTreeMap<Double, ShotData> activeMap = passing ? passingMap : shotMap;

    double tof = activeMap.get(targetToTurretDistance).tof();
    Pose2d predictedTurretPose = turretPosition;
    double predictedDistance = targetToTurretDistance;

    for (int i = 0; i < 5; i++) {
      tof = activeMap.get(predictedDistance).tof();
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
    ShotData shotData = activeMap.get(predictedDistance);
    double hoodAngleRots = shotData.getHoodAngleRots();
    double flywheelRPM = shotData.flywheelRPM() + flywheelOffset;
    double flywheelIdleRPM = MathUtil.clamp(flywheelRPM, 0.0, maxFlywheelIdleRPM.get());
    double backRollerRPM = shotData.backRollerRPM();

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
            backRollerRPM,
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
      double backRollerRPM,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  public record ShotData(
      double flywheelRPM, double hoodAngleDegs, double tof, double backRollerRPM) {
    public ShotData(double flywheelRPM, double hoodAngleDegs) {
      this(flywheelRPM, hoodAngleDegs, 0.95, Double.NaN);
    }

    public ShotData(double flywheelRPM, double hoodAngleDegs, double tof) {
      this(flywheelRPM, hoodAngleDegs, tof, Double.NaN);
    }

    public double getHoodAngleRots() {
      return Units.degreesToRotations(hoodAngleDegs());
    }

    public static ShotData interpolate(ShotData start, ShotData end, double t) {
      return new ShotData(
          MathUtil.interpolate(start.flywheelRPM, end.flywheelRPM, t),
          MathUtil.interpolate(start.hoodAngleDegs, end.hoodAngleDegs, t),
          MathUtil.interpolate(start.tof, end.tof, t),
          interpolateBackRollerRpm(start.backRollerRPM, end.backRollerRPM, t));
    }

    private static double interpolateBackRollerRpm(double start, double end, double t) {
      if (Double.isFinite(start) && Double.isFinite(end)) {
        return MathUtil.interpolate(start, end, t);
      }
      return Double.NaN;
    }
  }
}
