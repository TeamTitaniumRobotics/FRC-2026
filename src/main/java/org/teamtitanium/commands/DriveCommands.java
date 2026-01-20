package org.teamtitanium.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import org.teamtitanium.utils.TunerConstants;

public class DriveCommands {
  public static final double deadband = 0.1;

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(x, y);

    linearMagnitude =
        linearMagnitude * linearMagnitude; // Square the magnitude for finer control at low speeds

    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, deadband);
    return omega * omega * Math.signum(omega);
  }

  public static ChassisSpeeds getSpeedsFromJoysticks(
      double driverX, double driverY, double driverOmega) {
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(driverX, driverY)
            .times(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    double omega = getOmegaFromJoysticks(driverOmega);

    return new ChassisSpeeds(
        linearVelocity.getX(),
        linearVelocity.getY(),
        omega * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / Swerve.DRIVE_BASE_RADIUS);
  }

  public static Command joystickDrive(
      Swerve swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              getSpeedsFromJoysticks(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble());
          swerve.runVelocity(
              robotRelative.getAsBoolean()
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? RobotState.getInstance().getRotation().plus(Rotation2d.kPi)
                          : RobotState.getInstance().getRotation()));
        },
        swerve);
  }
}
