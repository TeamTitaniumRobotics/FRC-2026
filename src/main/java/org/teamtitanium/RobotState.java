package org.teamtitanium;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.FieldConstants;

public class RobotState {
  private static final double poseBufferSizeSeconds = 2.0;

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // Poses
  @Getter @Setter @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @Getter @Setter @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d gyroOffset = Rotation2d.kZero;

  @Getter
  @AutoLogOutput(key = "RobotState/RobotVelocity")
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  @Getter @Setter private Rotation2d pitch = Rotation2d.kZero;
  @Getter @Setter private Rotation2d roll = Rotation2d.kZero;

  private RobotState() {
    kinematics = new SwerveDriveKinematics(Swerve.getModuleTranslations());
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist2d = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist2d);
    // Use gyro angle if connected
    observation
        .gyroAngle()
        .ifPresent(
            gyroAngle -> {
              // Add offset to measured gyro angle
              Rotation2d angle = gyroAngle.plus(gyroOffset);
              odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
            });
    // Add pose to buffer at timestamp
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    // Update estimated pose from difference from last pose
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void addSwerveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public AngularVelocity getFlywheelSetpoint() {
    // Placeholder value; replace with actual logic to determine flywheel setpoint
    return RotationsPerSecond.of(0.0);
  }

  public Angle getHoodSetpoint() {
    // Placeholder value; replace with actual logic to determine hood setpoint
    return Rotations.of(0.0);
  }

  public Angle getTurretSetpoint() {
    // Placeholder value; replace with actual logic to determine turret setpoint
    // Use field position and robot velocity direction to determine whether to track hub or side of
    // zone for passing
    Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    var targetAngle = getEstimatedPose().getTranslation().minus(hub).getAngle();
    Logger.recordOutput("Turret/Sim/TargetAngle", targetAngle);
    return Radians.of(targetAngle.getRadians());
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
}
