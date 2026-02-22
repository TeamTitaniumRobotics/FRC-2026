package org.teamtitanium.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.teamtitanium.RobotState;
import org.teamtitanium.subsystems.swerve.Swerve;

public class DriveCommands {
  public static final double DEADBAND = 0.1;

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);

    // Prevent Rotation2d(0, 0) exception when stick is centered / in deadband
    if (linearMagnitude <= 1e-6) {
      return Translation2d.kZero;
    }

    Rotation2d linearDirection = new Rotation2d(x, y);

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  // public static double getOmegaFromJoysticks(double driverOmega) {
  //   double omega = MathUtil.applyDeadband(driverOmega, DEADBAND);
  //   return omega * omega * Math.signum(omega);
  // }

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

          double theta = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          theta = Math.copySign(theta * theta, theta);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * Swerve.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * Swerve.getMaxLinearSpeedMetersPerSec(),
                  theta * Swerve.getMaxAngularVelocityRadPerSec());

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
}
