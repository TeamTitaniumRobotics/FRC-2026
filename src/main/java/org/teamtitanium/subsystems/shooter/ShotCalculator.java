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

  private ShotParameters latestParameters = null;

  private static final double phaseDelay = 0.03; // Turret response delay

  private static final InterpolatingTreeMap<Double, ShotData> shotMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);
  private static final InterpolatingTreeMap<Double, ShotData> passingMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

  static {
    shotMap.put(1.25, new ShotData(2450, 3));
    shotMap.put(1.50, new ShotData(2500, 4));
    shotMap.put(2.00, new ShotData(2650, 6));
    shotMap.put(2.50, new ShotData(2850, 8, 1.1));
    shotMap.put(2.75, new ShotData(2950, 9, 1.1));
    shotMap.put(3.00, new ShotData(3050, 10, 1.1));
    shotMap.put(3.25, new ShotData(3150, 11, 1.1));
    shotMap.put(3.50, new ShotData(3250, 12, 1.1));
    shotMap.put(3.75, new ShotData(3350, 13, 1.1));
    shotMap.put(4.00, new ShotData(3450, 14, 1.12));
    shotMap.put(4.25, new ShotData(3550, 14.5, 1.12));
    shotMap.put(4.55, new ShotData(3700, 15, 1.12));
    shotMap.put(4.75, new ShotData(3775, 16, 1.14));
    shotMap.put(5.0, new ShotData(3950, 18, 1.14));

    passingMap.put(5.0, new ShotData(3350, 20.0, 1.2));
    passingMap.put(6.0, new ShotData(3600, 22.5, 1.2));
    passingMap.put(7.0, new ShotData(4150, 25.0, 1.25));
    passingMap.put(8.0, new ShotData(4650, 27.5, 1.25));
    passingMap.put(9.0, new ShotData(5650, 27.5, 1.3));
    passingMap.put(10.0, new ShotData(6650, 27.5, 1.3));
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

    Logger.recordOutput("ShotCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));

    Pose2d turretPosition = estimatedPose.transformBy(TURRET_TO_ROBOT.toTransform2d());
    double targetToTurretDistance = target.getDistance(turretPosition.getTranslation());
    Logger.recordOutput("ShotCalculator/TurretPose", turretPosition);

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
    double flywheelRPM = shotMap.get(predictedDistance).flywheelRPM() + flywheelOffset;
    double flywheelIdleRPM = MathUtil.clamp(flywheelRPM, 0.0, maxFlywheelIdleRPM.get());

    boolean isValid =
        FieldConstants.Zones.NO_SHOOT_ZONE
            .containsRobot(() -> RobotState.getInstance().getEstimatedPose(), false)
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

    Logger.recordOutput("ShotCalculator/PredictedPose", predictedRobotPose);
    Logger.recordOutput("ShotCalculator/PredictedDistance", predictedDistance);

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
