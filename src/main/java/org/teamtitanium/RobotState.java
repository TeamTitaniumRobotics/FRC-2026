package org.teamtitanium;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.AllianceFlipUtil;
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
  private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  // Odometry
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;
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
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, lastWheelPositions, Pose2d.kZero
            // VecBuilder.fill(0.05, 0.05, Math.toRadians(5.0)),
            // VecBuilder.fill(0.5, 0.5, Math.toRadians(30.0))
            );
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Twist2d twist2d = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();
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
    // Update estimated pose from estimator
    Rotation2d gyroAngle = observation.gyroAngle().orElse(estimatedPose.getRotation());
    poseEstimator.updateWithTime(observation.timestamp(), gyroAngle, observation.wheelPositions());
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(
      Pose2d visionPoseMeters, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
    poseEstimator.addVisionMeasurement(visionPoseMeters, timestampSeconds, visionStdDevs);
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setEstimatedPose(Pose2d pose) {
    estimatedPose = pose;
    poseEstimator.resetPosition(pose.getRotation(), lastWheelPositions, pose);
  }

  public Trigger inAllianceZone =
      new Trigger(
          () ->
              AllianceFlipUtil.apply(FieldConstants.Zones.allianceZone)
                  .contains(getEstimatedPose().getTranslation()));

  public Trigger inNeutralZone =
      new Trigger(
          () -> FieldConstants.Zones.neutralZone.contains(getEstimatedPose().getTranslation()));

  // TODO: Add a check for if robot is driving towards trench at speed and if so, stow hood and
  // align drivetrain with the trench. Also add an override on driver's controller to override the
  // function. Also add drivetrain auto rotate for bump with same override
  public Trigger underTrench =
      new Trigger(() -> false); // TODO: Replace with actual logic to determine if under trench

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
    Rotation2d targetAngle = hub.minus(getEstimatedPose().getTranslation()).getAngle();
    Logger.recordOutput("Turret/Sim/TargetAngle", targetAngle);
    return Radians.of(targetAngle.getRadians());
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}
}
