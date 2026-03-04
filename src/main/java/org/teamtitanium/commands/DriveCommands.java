package org.teamtitanium.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.FieldConstants;

public class DriveCommands {
  public static final double DEADBAND = 0.1;

  private static final PIDController TRENCH_CONTROLLER = new PIDController(8.0, 0.0, 0.05);
  private static final PIDController ROTATIONAL_CONTROLLER = new PIDController(5.0, 0.0, 0.0);

  static {
    TRENCH_CONTROLLER.setTolerance(Units.inchesToMeters(5.0));
    ROTATIONAL_CONTROLLER.setTolerance(Units.degreesToRadians(5.0));
    ROTATIONAL_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, DEADBAND);
    return omega = Math.copySign(omega * omega, omega);
  }

  // public static ChassisSpeeds getSpeedsFromJoysticks(
  //     double driverX, double driverY, double driverOmega) {
  //   Translation2d linearVelocity =
  //       getLinearVelocityFromJoysticks(driverX, driverY)
  //           .times(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  //   double omega = getOmegaFromJoysticks(driverOmega);

  //   return new ChassisSpeeds(
  //       linearVelocity.getX(),
  //       linearVelocity.getY(),
  //       omega * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / Swerve.DRIVE_BASE_RADIUS);
  // }

  public static Command joystickDrive(
      Swerve swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = getOmegaFromJoysticks(omegaSupplier.getAsDouble());

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * Swerve.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * Swerve.getMaxLinearSpeedMetersPerSec(),
                  omega * Swerve.getMaxAngularVelocityRadPerSec());

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          swerve.runVelocity(
              robotRelative.getAsBoolean()
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? RobotState.getInstance().getRotation().plus(Rotation2d.kPi)
                          : RobotState.getInstance().getRotation()));
        },
        swerve);
  }

  public static Command trenchDrive(Swerve swerve, DoubleSupplier xSupplier) {
    return Commands.run(
            () -> {
              double trenchOutput =
                  TRENCH_CONTROLLER.calculate(RobotState.getInstance().getEstimatedPose().getY());

              double omega =
                  ROTATIONAL_CONTROLLER.calculate(
                      RobotState.getInstance().getRotation().getRadians());

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xSupplier.getAsDouble() * Swerve.getMaxLinearSpeedMetersPerSec(),
                      trenchOutput,
                      omega);

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              swerve.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? RobotState.getInstance().getRotation().plus(Rotation2d.kPi)
                          : RobotState.getInstance().getRotation()));
            },
            swerve)
        .beforeStarting(
            () -> {
              TRENCH_CONTROLLER.setSetpoint(
                  RobotState.getInstance().getEstimatedPose().getY()
                          < FieldConstants.fieldWidth / 2.0
                      ? FieldConstants.LinesHorizontal.rightTrenchOpenMiddle
                      : FieldConstants.LinesHorizontal.leftTrenchOpenMiddle);
              ROTATIONAL_CONTROLLER.setSetpoint(getTrenchAngleRad());
            });
  }

  private static double getTrenchAngleRad() {
    double angle = RobotState.getInstance().getRotation().getRadians() % (2 * Math.PI);
    if (angle < 0) {
      angle += 2 * Math.PI;
    }

    int nearestQuarter = (int) Math.round(angle / (Math.PI / 2.0)) % 4;
    double nearestAngle = nearestQuarter * (Math.PI / 2.0);

    Logger.recordOutput("Swerve/Trench/NearestAngle", Radians.of(nearestAngle));

    if (nearestAngle < -Math.PI) {
      nearestAngle += 2 * Math.PI;
    }
    if (nearestAngle > Math.PI) {
      nearestAngle -= 2 * Math.PI;
    }
    return nearestAngle;
  }
}
