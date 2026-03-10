package org.teamtitanium;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.NoSuchElementException;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.utils.FieldConstants;

public class RobotState {
  private static final double poseBufferSizeSeconds = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

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
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

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
    for (int i = 0; i < 3; i++) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = new SwerveDriveKinematics(Swerve.getModuleTranslations());
    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, lastWheelPositions, Pose2d.kZero
            // VecBuilder.fill(0.05, 0.05, Math.toRadians(5.0)),
            // VecBuilder.fill(0.5, 0.5, Math.toRadians(30.0))
            );
  }

  public void addOdometryObservation(OdometryObservation observation) {
    // Update odometry pose
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

    // Apply odometry data to estimated pose
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param observation The vision observation containing the vision pose, timestamp, and standard
   *     deviations of the vision measurement.
   */
  public void addVisionMeasurement(VisionObservation observation) {
    // Check if the observation is too old to be used
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException e) {
      return;
    }

    // Get the odometry pose at the time of the vision observation
    var sampledPose = poseBuffer.getSample(observation.timestamp());
    if (sampledPose.isEmpty()) {
      // Return if there is no sampled pose
      return;
    }

    // Calculate the transform from the odometry pose to the vision pose
    Transform2d sampledToOdometryTransform = new Transform2d(sampledPose.get(), odometryPose);
    // Transform2d odometryToSampledTransform = sampledToOdometryTransform.inverse(); // TODO: Test
    // with inverse
    Transform2d odometryToSampledTransform = new Transform2d(odometryPose, sampledPose.get());

    // Apply the odometry transform to the estimated pose to get the estimated pose
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampledTransform);

    // Calculate the 3 x 3 Kalman gain matrix for the vision measurement
    double[] r = new double[3];
    for (int i = 0; i < 3; i++) {
      r[i] = Math.pow(observation.visionStdDevs().get(i, 0), 2);
    }

    // Kalman gain is calculated as K = Q / (Q + R) where Q is the process noise covariance and R is
    // the measurement noise covariance. Since Q and R are diagonal matrices, we can calculate K for
    // each state independently. If the standard deviation of the measurement is zero, we set the
    // Kalman gain to zero to avoid division by zero and to indicate that we have no confidence in
    // the measurement for that state.
    var visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < r.length; row++) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    // Apply the Kalman gain to the transform to get the scaled transform
    Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

    // Scale the transform by the Kalman gain
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            new Rotation2d(kTimesTransform.get(2, 0)));

    // Update the estimated pose by applying the scaled transform to the estimate at the time of the
    // vision measurement and then applying the transform from the sampled pose to the odometry pose
    // to account for the difference between the sampled pose and the current odometry pose.
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampledToOdometryTransform);
  }

  public Trigger inAllianceZone =
      FieldConstants.Zones.ALLIANCE_ZONE.containsRobot(this::getEstimatedPose, false);

  public Trigger inNeutralZone =
      FieldConstants.Zones.NEUTRAL_ZONE.containsRobot(this::getEstimatedPose, true);

  // TODO: Add a check for if robot is driving towards trench at speed and if so, stow hood and
  // align drivetrain with the trench. Also add an override on driver's controller to override the
  // function. Also add drivetrain auto rotate for bump with same override
  @AutoLogOutput(key = "RobotState/UnderTrench")
  public Trigger underTrench =
      FieldConstants.Zones.TRENCH_ZONE.contains(() -> getEstimatedPose().getTranslation());

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void addSwerveSpeeds(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRotation());
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  public record VisionObservation(
      Pose3d visionPose, double timestamp, Matrix<N3, N1> visionStdDevs) {}
}
