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
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.utils.AllianceFlipUtil;
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
      new LoggedTunableNumber("ShotCalculator/MaxFlywheelIdleRPM", 1800.0);

  private ShotParameters latestParameters = null;

  private static final double phaseDelay = 0.03; // Turret response delay

  private static final InterpolatingTreeMap<Double, ShotData> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);
  private static final InterpolatingTreeMap<Double, ShotData> passingMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

  static {
    shotMap.put(1.35, new ShotData(2350, 3));
    shotMap.put(1.50, new ShotData(2400, 4));
    shotMap.put(2.00, new ShotData(2600, 6));
    shotMap.put(2.50, new ShotData(2875, 8));
    shotMap.put(2.75, new ShotData(2900, 9));
    shotMap.put(3.05, new ShotData(3000, 10));
    shotMap.put(3.25, new ShotData(3100, 11));
    shotMap.put(3.50, new ShotData(3100, 12));
    shotMap.put(3.75, new ShotData(3225, 13));
    shotMap.put(4.05, new ShotData(3350, 13));
    shotMap.put(4.35, new ShotData(3500, 13));
    shotMap.put(4.5, new ShotData(3600, 14));
    shotMap.put(5.0, new ShotData(3700, 16));

    passingMap.put(4.5, new ShotData(4250, 22.5));
  }

  public ShotParameters getParameters() {
    boolean passing = !RobotState.getInstance().inAllianceZone.getAsBoolean();
    // if (AllianceFlipUtil.shouldFlip()) {
    //   passing =
    //       RobotState.getInstance().getEstimatedPose().getX()
    //           < AllianceFlipUtil.applyX(FieldConstants.LinesVertical.hubCenter);
    // } else {
    //   passing =
    //       RobotState.getInstance().getEstimatedPose().getX()
    //           > FieldConstants.LinesVertical.hubCenter;
    // }

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

    Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));

    Pose2d turretPosition = estimatedPose.transformBy(TURRET_TO_ROBOT.toTransform2d());
    double targetToTurretDistance = target.getDistance(turretPosition.getTranslation());

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
    double hoodAngleRots = shotMap.get(predictedDistance).getHoodAngleRots();
    double flywheelRPM = shotMap.get(predictedDistance).flywheelRPM();
    double flywheelIdleRPM = MathUtil.clamp(flywheelRPM, 0.0, maxFlywheelIdleRPM.get());

    latestParameters =
        new ShotParameters(
            true,
            turretAngle.getRotations(),
            hoodAngleRots,
            flywheelRPM,
            flywheelIdleRPM,
            predictedDistance,
            targetToTurretDistance,
            tof,
            passing);

    Logger.recordOutput("ShotCalculator/PredictedPose", predictedRobotPose);
    Logger.recordOutput("ShotCalculator/PredictedDistance", predictedDistance);

    return latestParameters;
  }

  private static Rotation2d getTurretAngle(Pose2d robotPose, Translation2d target) {
    Rotation2d robotToTarget = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d targetAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    TURRET_TO_ROBOT.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d turretAngle = robotToTarget.minus(robotPose.getRotation()).minus(targetAngle);

    return turretAngle;
  }

  public void resetShotParameters() {
    latestParameters = null;
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
