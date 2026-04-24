package org.teamtitanium.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.Robot;
import org.teamtitanium.RobotState;
import org.teamtitanium.RobotState.OdometryObservation;
import org.teamtitanium.utils.AllianceFlipUtil;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableBoolean;
import org.teamtitanium.utils.LoggedTunableNumber;
import org.teamtitanium.utils.TunerConstants;
import org.teamtitanium.utils.swerve.ModuleLimits;
import org.teamtitanium.utils.swerve.SwerveSetpoint;
import org.teamtitanium.utils.swerve.SwerveSetpointGenerator;

public class Swerve extends SubsystemBase {
  public static final LoggedTunableBoolean useSwerveSetpointGenerator =
      new LoggedTunableBoolean("Swerve/UseSwerveSetpointGenerator", false);

  public static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationX),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public static final Lock odometryLock = new ReentrantLock();

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Debouncer gyroDisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert gyroDisconnectedAlert =
      new Alert("Swerve Gyro Disconnected", Alert.AlertType.kError);

  private final SwerveModule[] swerveModules = new SwerveModule[4];

  private final SysIdRoutine sysId;
  private static final int FAST_LOG_DIVISOR = 5;

  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Swerve/CoastWaitTimeSecs", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecThreshold =
      new LoggedTunableNumber("Swerve/CoastMetersPerSecThreshold", 0.05);

  private final Timer lastMovementTimer = new Timer();

  @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.ALWAYS_BRAKE;
  @AutoLogOutput private boolean brakeModeEnabled = true;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator swerveSetpointGenerator =
      new SwerveSetpointGenerator(kinematics, getModuleTranslations());

  private final double maxLinearAcceleration = 22.0;
  private final double maxAngularVelocity = Units.degreesToRadians(1080.0);
  private final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          maxLinearAcceleration,
          maxAngularVelocity);

  @AutoLogOutput private boolean velocityMode = false;

  private final PIDController xPosController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yPosController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  public final FollowPath.Builder pathBuilder =
      new FollowPath.Builder(
              this,
              () -> RobotState.getInstance().getEstimatedPose(),
              () -> RobotState.getInstance().getRobotVelocity(),
              (speeds) -> this.runVelocity(speeds),
              new PIDController(5.0, 0.0, 0.0),
              new PIDController(3.0, 0.0, 0.0),
              new PIDController(2.0, 0.0, 0.0))
          .withDefaultShouldFlip();

  private int velocityLogCounter = 0;
  private int choreoLogCounter = 0;

  public Swerve(
      GyroIO gyroIO,
      SwerveModuleIO flModuleIO,
      SwerveModuleIO frModuleIO,
      SwerveModuleIO blModuleIO,
      SwerveModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    this.swerveModules[0] = new SwerveModule(flModuleIO, 0, TunerConstants.FrontLeft);
    this.swerveModules[1] = new SwerveModule(frModuleIO, 1, TunerConstants.FrontRight);
    this.swerveModules[2] = new SwerveModule(blModuleIO, 2, TunerConstants.BackLeft);
    this.swerveModules[3] = new SwerveModule(brModuleIO, 3, TunerConstants.BackRight);

    lastMovementTimer.start();
    setBrakeMode(true);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      config =
          new RobotConfig(
              Pounds.of(150),
              KilogramSquareMeters.of(6.0),
              new ModuleConfig(
                  TunerConstants.FrontLeft.WheelRadius,
                  TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                  1.2,
                  DCMotor.getKrakenX60(1),
                  TunerConstants.FrontLeft.SlipCurrent,
                  1),
              getModuleTranslations());
    }

    AutoBuilder.configure(
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().setEstimatedPose(pose),
        () -> getChassisSpeeds(),
        (speeds, feedforwards) -> this.runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(6.35, 0.0, 0.1), new PIDConstants(5.25, 0.0, 0.1)),
        config,
        () -> AllianceFlipUtil.shouldFlip(),
        this);

    Path.setDefaultGlobalConstraints(
        new Path.DefaultGlobalConstraints(4.5, 10.0, 600, 2000, 0.03, 2.0, 0.25));

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Swerve/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    PhoenixOdometryThread.getInstance().start();
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);

      for (SwerveModule module : swerveModules) {
        module.updateInputs();
      }
    } finally {
      odometryLock.unlock();
    }

    Logger.processInputs("Swerve/Gyro", gyroInputs);
    for (var swerveModule : swerveModules) {
      swerveModule.logInputs();
      swerveModule.periodic();
    }
    LoggedTracer.record("Swerve/Inputs");

    // Stop swerve modules if disabled
    if (DriverStation.isDisabled()) {
      for (var swerveModule : swerveModules) {
        swerveModule.stop();
      }
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/SwerveStates/SetpointsUnoptimized", new SwerveModuleState[] {});
    }

    // Update robot state with odometry updates
    double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      var wheelPositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = swerveModules[j].getOdometryPositions()[i];
        moduleDeltas[j] =
            new SwerveModulePosition(
                wheelPositions[j].distanceMeters - lastModulePositions[j].distanceMeters,
                wheelPositions[j].angle);
        lastModulePositions[j] = wheelPositions[j];
      }
      if (gyroInputs.connected) {
        rawGyroRotation = Rotation2d.fromRadians(gyroInputs.odometryYawPositionsRads[i]);
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      RobotState.getInstance()
          .addOdometryObservation(
              new OdometryObservation(
                  wheelPositions, Optional.ofNullable(rawGyroRotation), sampleTimestamps[i]));

      // // Log 3D estimated pose with pitch and roll adjustments TODO: Move to RobotState
      // Logger.recordOutput(
      //     "RobotState/EstimatedPose3d",
      //     new Pose3d(RobotState.getInstance().getEstimatedPose())
      //         .exp(
      //             new Twist3d(
      //                 0.0,
      //                 0.0,
      //                 Math.abs(gyroInputs.pitchPositionRads) *
      // TunerConstants.FrontLeft.LocationX,
      //                 0.0,
      //                 gyroInputs.pitchPositionRads,
      //                 0.0))
      //         .exp(
      //             new Twist3d(
      //                 0.0,
      //                 0.0,
      //                 Math.abs(gyroInputs.rollPositionRads) * TunerConstants.FrontLeft.LocationY,
      //                 gyroInputs.rollPositionRads,
      //                 0.0,
      //                 0.0)));
    }

    RobotState.getInstance().addSwerveSpeeds(getChassisSpeeds());
    RobotState.getInstance().setPitch(Rotation2d.fromRadians(gyroInputs.pitchPositionRads));
    RobotState.getInstance().setRoll(Rotation2d.fromRadians(gyroInputs.rollPositionRads));

    // Update break mode based on coast request
    if (DriverStation.isEnabled()) {
      coastRequest = CoastRequest.ALWAYS_BRAKE;
    }

    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else {
          if (Arrays.stream(swerveModules)
              .anyMatch(
                  swerveModule ->
                      Math.abs(swerveModule.getVelocityMetersPerSec())
                          > coastMetersPerSecThreshold.get())) {
            lastMovementTimer.reset();
          }
          if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
            setBrakeMode(false);
          }
        }
      }
      case ALWAYS_BRAKE -> setBrakeMode(true);
      case ALWAYS_COAST -> setBrakeMode(false);
    }

    // Update current setpoint if not in velocity mode
    if (!velocityMode) {
      currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    }

    // Update gyro disconnected alert
    gyroDisconnectedAlert.set(
        !gyroDisconnectedDebouncer.calculate(gyroInputs.connected)
            && Constants.getMode() != Constants.Mode.SIM
            && !Robot.isInitializing());

    // Record swerve periodic cycle time
    LoggedTracer.record("Swerve/Periodic");
  }

  public void runVelocity(ChassisSpeeds speeds) {
    velocityMode = true;
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    boolean shouldLogFastData = velocityLogCounter++ % FAST_LOG_DIVISOR == 0;
    if (Constants.tuningMode) {
      shouldLogFastData = true;
    }
    if (shouldLogFastData) {
      Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", speeds);
      Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
    }

    for (int i = 0; i < 4; i++) {
      swerveModules[i].runSetpoint(setpointStates[i]);
    }

    if (shouldLogFastData) {
      Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", setpointStates);
    }
  }

  // public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
  //   velocityMode = true;
  //   ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
  //   SwerveModuleState[] setpointStatesUnoptimized =
  // kinematics.toSwerveModuleStates(discreteSpeeds);
  //   currentSetpoint =
  //       swerveSetpointGenerator.generateSetpoint(
  //           moduleLimitsFree, currentSetpoint, discreteSpeeds, Constants.loopPeriodSecs);
  //   SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

  //   Logger.recordOutput("Swerve/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
  //   Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
  //   Logger.recordOutput("Swerve/ChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

  //   SwerveModuleState[] wheelForces = new SwerveModuleState[4];
  //   SwerveModuleState[] moduleStates = getModuleStates();
  //   for (int i = 0; i < 4; i++) {
  //     Rotation2d wheelAngle = moduleStates[i].angle;
  //     setpointStates[i].optimize(wheelAngle);
  //     setpointStates[i].cosineScale(wheelAngle);

  //     var wheelForce = moduleForces.get(i);
  //     Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
  //     double wheelTorqueNm = wheelForce.dot(wheelDirection) *
  // TunerConstants.FrontLeft.WheelRadius;
  //     swerveModules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

  //     wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
  //   }
  //   Logger.recordOutput("Swerve/SwerveStates/ModuleForces", wheelForces);
  // }

  /** Follows a Choreo trajectory. */
  public void followChoreoTrajectory(SwerveSample sample) {
    Pose2d pose = RobotState.getInstance().getEstimatedPose();

    boolean shouldLogChoreoData = choreoLogCounter++ % FAST_LOG_DIVISOR == 0;
    if (Constants.tuningMode) {
      shouldLogChoreoData = true;
    }
    if (shouldLogChoreoData) {
      Logger.recordOutput("Swerve/Choreo/Sample", sample);
      Logger.recordOutput("Swerve/Choreo/ErrorX", sample.x - pose.getX());
      Logger.recordOutput("Swerve/Choreo/ErrorY", sample.y - pose.getY());
      Logger.recordOutput(
          "Swerve/Choreo/ErrorTheta", sample.heading - pose.getRotation().getRadians());
    }

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            sample.vx + xPosController.calculate(pose.getX(), sample.x),
            sample.vy + yPosController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, pose.getRotation());

    runVelocity(robotRelativeSpeeds);
  }

  public void runCharacterization(double output) {
    velocityMode = false;
    for (var swerveModule : swerveModules) {
      swerveModule.runCharacterization(output);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
    for (int i = 0; i < 4; i++) {
      states[i].optimize(swerveModules[i].getAngle());
      swerveModules[i].runSetpoint(states[i]);
    }
  }

  public void setGyroAngle(Angle angle) {
    gyroIO.setGyroAngle(angle);
  }

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(swerveModules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  @AutoLogOutput(key = "Swerve/ChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public static double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public static double getMaxAngularVelocityRadPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / DRIVE_BASE_RADIUS;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += swerveModules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the positions of the modules in radians for wheel radius characterization. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] positions = new double[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = swerveModules[i].getWheelRadiusCharacterizationPosition();
    }
    return positions;
  }

  /** Returns the translations of the modules in meters. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }
}
