package org.teamtitanium;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.teamtitanium.subsystems.swerve.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import lombok.Getter;
import lombok.Setter;

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
    @Getter
    @AutoLogOutput
    private Pose2d odometryPose = Pose2d.kZero;
    @Getter
    @AutoLogOutput
    private Pose2d estimatedPose = Pose2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

    // Odometry
    private final SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d gyroOffset = Rotation2d.kZero;

    @Getter @AutoLogOutput(key = "RobotState/RobotVelocity")
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
        observation.gyroAngle().ifPresent(gyroAngle -> {
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

    public void addSwerveSpeeds(ChassisSpeeds speeds) {
        robotVelocity = speeds;
    }

    public record OdometryObservation(SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle,
            double timestamp) {
    }
}
