package org.teamtitanium.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.LoggedTunableNumber;
import org.teamtitanium.utils.math.GeomUtil;

public class DriveToPose {
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("DriveToPose/DrivekP", 5.0);
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("DriveToPose/DrivekD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveToPose/ThetakP", 4.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveToPose/ThetakD", 0.0);
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity", 3.8);
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration", 3.0);
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity", Units.degreesToRadians(360));
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration", Units.degreesToRadians(360));
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance", 0.04);
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance", Units.degreesToRadians(4.0));
  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("DriveToPose/FFMinRadius", 0.01);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("DriveToPose/FFMaxRadius", 0.05);

  private static final ProfiledPIDController driveController =
      new ProfiledPIDController(
          drivekP.get(),
          0,
          drivekD.get(),
          new Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
  private static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetakP.get(),
          0,
          thetakD.get(),
          new Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));

  static {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static Translation2d lastSetpointTranslation = new Translation2d();
  private static double driveError = 0.0;
  private static double thetaError = 0.0;

  public static Command driveToPose(Supplier<Pose2d> targetPoseSupplier, Swerve swerve) {
    return Commands.run(
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              Pose2d targetPose = targetPoseSupplier.get();

              double currentError =
                  currentPose.getTranslation().getDistance(targetPose.getTranslation());

              double ffScalar =
                  MathUtil.clamp(
                      (currentError - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                      0.0,
                      1.0);

              driveError = currentError;

              driveController.reset(
                  lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                  driveController.getSetpoint().velocity);

              double desiredDriveVelocity =
                  driveController.getSetpoint().velocity * ffScalar
                      + driveController.calculate(driveError, 0.0);

              if (currentError < driveController.getPositionTolerance()) {
                desiredDriveVelocity = 0.0;
              }

              lastSetpointTranslation =
                  new Pose2d(
                          targetPose.getTranslation(),
                          currentPose
                              .getTranslation()
                              .minus(targetPose.getTranslation())
                              .getAngle())
                      .transformBy(
                          GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                      .getTranslation();

              double thetaVelocity =
                  thetaController.getSetpoint().velocity * ffScalar
                      + thetaController.calculate(
                          currentPose.getRotation().getRadians(),
                          targetPose.getRotation().getRadians());

              if (thetaError < thetaController.getPositionTolerance()) {
                thetaVelocity = 0.0;
              }

              Translation2d driveVelocity =
                  new Pose2d(
                          new Translation2d(),
                          currentPose
                              .getTranslation()
                              .minus(targetPose.getTranslation())
                              .getAngle())
                      .transformBy(GeomUtil.toTransform2d(desiredDriveVelocity, 0.0))
                      .getTranslation();

              swerve.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      driveVelocity.getX(),
                      driveVelocity.getY(),
                      thetaVelocity,
                      currentPose.getRotation()));

              Logger.recordOutput("DriveToPose/DistanceMeasured", currentError);
              Logger.recordOutput(
                  "DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
              Logger.recordOutput(
                  "DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
              Logger.recordOutput(
                  "DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
              Logger.recordOutput(
                  "DriveToPose/Setpoint",
                  new Pose2d[] {
                    new Pose2d(
                        lastSetpointTranslation,
                        Rotation2d.fromRadians(thetaController.getSetpoint().position))
                  });
              Logger.recordOutput("DriveToPose/TargetPose", new Pose2d[] {targetPose});
            })
        .beforeStarting(
            () -> {
              Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
              ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
              Translation2d linearFieldVelocity =
                  new Translation2d(
                      fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

              driveController.reset(
                  currentPose
                      .getTranslation()
                      .getDistance(targetPoseSupplier.get().getTranslation()),
                  Math.min(
                      0.0,
                      -linearFieldVelocity
                          .rotateBy(
                              targetPoseSupplier
                                  .get()
                                  .getTranslation()
                                  .minus(currentPose.getTranslation())
                                  .getAngle()
                                  .unaryMinus())
                          .getX()));
              thetaController.reset(
                  currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);

              lastSetpointTranslation = currentPose.getTranslation();
            })
        .until(() -> driveController.atGoal() && thetaController.atGoal())
        .andThen(Commands.runOnce(() -> swerve.stop()));
  }
}
