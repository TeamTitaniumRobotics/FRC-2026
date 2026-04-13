package org.teamtitanium.utils.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants;

/**
 * Custom odometry class for swerve drive robots that calculates the robot's position based on
 * individual module displacements, treating each module's movement as an arc instead of a straight
 * line.
 *
 * <p>Inspired by FRC Team 581's implementation of FRC Team 1690's custom odometry
 */
public class CustomOdometry extends SwerveDriveOdometry {
  private static final int odometryLogDivisor = 10;

  private final int numberOfModules;
  private final Translation2d[] robotRelativeModuleOffsets;

  private Pose2d robotPose = Pose2d.kZero;
  private final SwerveModulePosition[] previousWheelPositions;
  private int odometryLogCounter = 0;

  public CustomOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions) {
    super(kinematics, gyroAngle, modulePositions);
    this.numberOfModules = kinematics.getModules().length;
    robotRelativeModuleOffsets = kinematics.getModules();

    previousWheelPositions = new SwerveModulePosition[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      previousWheelPositions[i] = new SwerveModulePosition();
    }
  }

  /**
   * Calculates the displacement of a swerve module based on its previous and current positions as
   * an arc movement.
   *
   * @param previousWheelPosition The previous position of the swerve module
   * @param currentWheelPosition The current position of the swerve module
   * @return The displacement of the swerve module as a Translation2d
   */
  private static Translation2d getModuleDisplacement(
      SwerveModulePosition previousWheelPosition, SwerveModulePosition currentWheelPosition) {
    // Calculate the difference between current and previous wheel angles and
    // distances
    double differenceVectorAngle =
        currentWheelPosition.angle.getRadians() - previousWheelPosition.angle.getRadians();
    double differenceVectorLength =
        currentWheelPosition.distanceMeters - previousWheelPosition.distanceMeters;

    // If there is no change in angle, return the straight-line displacement
    if (differenceVectorAngle == 0) {
      return new Translation2d(differenceVectorLength, currentWheelPosition.angle);
    }

    // Calculate the radius of the arc traveled by the wheel. Positive = left turn,
    // negative = right turn
    double radius = differenceVectorLength / differenceVectorAngle;
    if (Constants.tuningMode) {
      Logger.recordOutput("CustomOdometry/GetModuleDisplacement/Radius", radius);
    }

    // Calculate the center of the circle that the wheel is turning around
    // and the displacement from the previous position to the current position
    // The previous position is treated as the "0 degree" point on the circle
    // It is also always perpendicular to the previous wheel angle
    double circleCenterX = -radius * previousWheelPosition.angle.getSin();
    double circleCenterY = radius * previousWheelPosition.angle.getCos();

    // Calculate the displacement from the center of the circle to the current
    // position
    double displacementX = circleCenterX + radius * currentWheelPosition.angle.getSin();
    double displacementY = circleCenterY - radius * currentWheelPosition.angle.getCos();

    return new Translation2d(displacementX, displacementY);
  }

  /**
   * Updates the robot's pose based on the current gyro angle and module positions.
   *
   * @param gyroAngle The current gyro angle of the robot
   * @param modulePositions The current positions of the swerve modules
   * @return The updated pose of the robot
   */
  @Override
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    // Calculate the field-relative poses of each module based on the robot's
    // current pose
    Pose2d[] fieldRelativeModulePosesOfPreviousPose = new Pose2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      fieldRelativeModulePosesOfPreviousPose[i] =
          robotPose.transformBy(new Transform2d(robotRelativeModuleOffsets[i], Rotation2d.kZero));
    }

    // Calculate the displacement of each module
    Translation2d[] moduleDisplacements = new Translation2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      moduleDisplacements[i] = getModuleDisplacement(previousWheelPositions[i], modulePositions[i]);
    }

    // Convert module displacements to field-relative displacements
    Translation2d[] fieldRelativeModuleDisplacements = new Translation2d[numberOfModules];
    for (int i = 0; i < numberOfModules; i++) {
      fieldRelativeModuleDisplacements[i] =
          fieldRelativeModulePosesOfPreviousPose[i]
              .transformBy(new Transform2d(moduleDisplacements[i], Rotation2d.kZero))
              .getTranslation();
    }

    // Average the field-relative module displacements to get the updated robot pose
    Translation2d sumOfFieldRelativeModuleDisplacements = new Translation2d();
    for (int i = 0; i < numberOfModules; i++) {
      sumOfFieldRelativeModuleDisplacements =
          sumOfFieldRelativeModuleDisplacements.plus(fieldRelativeModuleDisplacements[i]);
    }
    double updatedPoseX = sumOfFieldRelativeModuleDisplacements.getX() / numberOfModules;
    double updatedPoseY = sumOfFieldRelativeModuleDisplacements.getY() / numberOfModules;
    Pose2d updatedPose = new Pose2d(updatedPoseX, updatedPoseY, gyroAngle);

    boolean shouldLogDetailedData = odometryLogCounter++ % odometryLogDivisor == 0;
    if (Constants.tuningMode) {
      shouldLogDetailedData = true;
    }
    if (shouldLogDetailedData) {
      // Logging for debugging, will remove if this works later
      Logger.recordOutput("CustomOdometry/UpdatedPose", updatedPose);
      Logger.recordOutput("CustomOdometry/PreviousPose", robotPose);
      Logger.recordOutput("CustomOdometry/PreviousWheelPositions", previousWheelPositions);
      Logger.recordOutput("CustomOdometry/CurrentWheelPositions", modulePositions);
      Logger.recordOutput("CustomOdometry/ModuleDisplacements", moduleDisplacements);
      for (int i = 0; i < numberOfModules; i++) {
        Logger.recordOutput(
            "CustomOdometry/FieldRelativeModuleDisplacements/" + i,
            new Pose2d(
                fieldRelativeModuleDisplacements[i], gyroAngle.plus(modulePositions[i].angle)));
      }
    }

    // Update the robot pose and previous wheel positions for the next update
    robotPose = updatedPose;
    for (int i = 0; i < numberOfModules; i++) {
      previousWheelPositions[i] = modulePositions[i];
    }

    return robotPose;
  }

  @Override
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    robotPose = new Pose2d(pose.getTranslation(), gyroAngle);
  }

  @Override
  public void resetPose(Pose2d poseMeters) {
    robotPose = poseMeters;
  }

  @Override
  public void resetTranslation(Translation2d translation) {
    robotPose = new Pose2d(translation, robotPose.getRotation());
  }

  @Override
  public void resetRotation(Rotation2d rotation) {
    robotPose = new Pose2d(robotPose.getTranslation(), rotation);
  }

  @Override
  public Pose2d getPoseMeters() {
    return robotPose;
  }
}
