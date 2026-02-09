package org.teamtitanium.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Setter;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.Robot;
import org.teamtitanium.RobotState;
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

  private static final Mass ROBOT_MASS = Pounds.of(150);
  private static final double WHEEL_COF = 1.2;
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(ROBOT_MASS)
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  7.363636363636365,
                  15.42857142857143,
                  Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                  Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                  Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                  WHEEL_COF));

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] swerveModules = new SwerveModule[4];
  private final SysIdRoutine sysId;
  private final Debouncer gyroDisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert gyroDisconnectedAlert =
      new Alert("Swerve Gyro Disconnected", Alert.AlertType.kError);

  private static final LoggedTunableNumber coastWaitTime =
      new LoggedTunableNumber("Swerve/CoastWaitTimeSecs", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecThreshold =
      new LoggedTunableNumber("Swerve/CoastMetersPerSecThreshold", 0.05);

  private final Timer lastMovementTimer = new Timer();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());

  @AutoLogOutput private boolean velocityMode = false;
  @AutoLogOutput private boolean brakeModeEnabled = true;

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private final SwerveSetpointGenerator swerveSetpointGenerator;

  private final double maxLinearAcceleration = 22.0;
  private final double maxAngularVelocity = Units.degreesToRadians(1080.0);
  private final ModuleLimits moduleLimitsFree =
      new ModuleLimits(
          TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          maxLinearAcceleration,
          maxAngularVelocity);

  @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.ALWAYS_BRAKE;

  private final PIDController xPosController =
      new PIDController(3.0, 0.0, 0.0); // TODO: Tune these PID values
  private final PIDController yPosController =
      new PIDController(3.0, 0.0, 0.0); // TODO: Tune these PID values
  private final PIDController headingController =
      new PIDController(3.0, 0.0, 0.0); // TODO: Tune these PID values

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

    swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics, getModuleTranslations());

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
    odometryLock.lock(); // Lock odometry for the duration of input updates
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (SwerveModule module : swerveModules) {
      module.updateInputs();
    }

    odometryLock.unlock();
    LoggedTracer.record("Swerve/Inputs");

    for (var swerveModule : swerveModules) {
      swerveModule.periodic();
    }

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
    double[] sampleTimestamps =
        Constants.getMode() == Constants.Mode.SIM
            ? new double[] {Timer.getTimestamp()}
            : gyroInputs.odometryYawTimestamps;
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      var wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = swerveModules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryObservation(
              new RobotState.OdometryObservation(
                  wheelPositions,
                  Optional.ofNullable(
                      gyroInputs.gyroData.connected()
                          ? Rotation2d.fromRadians(gyroInputs.odometryYawPositionsRads[i])
                          : null),
                  sampleTimestamps[i]));

      // Log 3D estimated pose with pitch and roll adjustments
      Logger.recordOutput(
          "RobotState/EstimatedPose3d",
          new Pose3d(RobotState.getInstance().getEstimatedPose())
              .exp(
                  new Twist3d(
                      0.0,
                      0.0,
                      Math.abs(gyroInputs.gyroData.pitchPositionRads())
                          * TunerConstants.FrontLeft.LocationX,
                      0.0,
                      gyroInputs.gyroData.pitchPositionRads(),
                      0.0))
              .exp(
                  new Twist3d(
                      0.0,
                      0.0,
                      Math.abs(gyroInputs.gyroData.rollPositionRads())
                          * TunerConstants.FrontLeft.LocationY,
                      gyroInputs.gyroData.rollPositionRads(),
                      0.0,
                      0.0)));
    }

    RobotState.getInstance().addSwerveSpeeds(getChassisSpeeds());
    RobotState.getInstance()
        .setPitch(Rotation2d.fromRadians(gyroInputs.gyroData.pitchPositionRads()));
    RobotState.getInstance()
        .setRoll(Rotation2d.fromRadians(gyroInputs.gyroData.rollPositionRads()));

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
        !gyroDisconnectedDebouncer.calculate(gyroInputs.gyroData.connected())
            && Constants.getMode() != Constants.Mode.SIM
            && !Robot.isInitializing());

    // Record swerve periodic cycle time
    LoggedTracer.record("Swerve/Periodic");
  }

  public void runVelocity(ChassisSpeeds speeds) {
    velocityMode = true;
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveModuleState[] setpointStates = setpointStatesUnoptimized;
    // if (useSwerveSetpointGenerator.get()) {
    //   currentSetpoint =
    //       swerveSetpointGenerator.generateSetpoint(
    //           moduleLimitsFree, currentSetpoint, discreteSpeeds, Constants.loopPeriodSecs);
    //   setpointStates = currentSetpoint.moduleStates();
    //   Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints",
    // currentSetpoint.chassisSpeeds());
    // }

    Logger.recordOutput("Swerve/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);

    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < 4; i++) {
      Rotation2d wheelAngle = moduleStates[i].angle;
      setpointStates[i].optimize(wheelAngle);
      setpointStates[i].cosineScale(wheelAngle);

      swerveModules[i].runSetpoint(setpointStates[i]);
    }
  }

  public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
    velocityMode = true;
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            moduleLimitsFree, currentSetpoint, discreteSpeeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    Logger.recordOutput("Swerve/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    SwerveModuleState[] wheelForces = new SwerveModuleState[4];
    SwerveModuleState[] moduleStates = getModuleStates();
    for (int i = 0; i < 4; i++) {
      Rotation2d wheelAngle = moduleStates[i].angle;
      setpointStates[i].optimize(wheelAngle);
      setpointStates[i].cosineScale(wheelAngle);

      var wheelForce = moduleForces.get(i);
      Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
      double wheelTorqueNm = wheelForce.dot(wheelDirection) * TunerConstants.FrontLeft.WheelRadius;
      swerveModules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

      wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
    }
    Logger.recordOutput("Swerve/SwerveStates/ModuleForces", wheelForces);
  }

  public void runCharacterization(double output) {
    velocityMode = false;
    for (var swerveModule : swerveModules) {
      swerveModule.runCharacterization(output);
    }
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

  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(swerveModules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  public void followChoreoTrajectory(SwerveSample sample) {
    Pose2d pose = RobotState.getInstance().getEstimatedPose(); // Ensure odometry is up to date by getting pose

    // Generate the desired speeds to follow that trajectory
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx
                + xPosController.calculate(
                    pose.getX(), sample.x),
            sample.vy
                + yPosController.calculate(
                    pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(
                    pose.getRotation().getRadians(),
                    sample.heading));

    // Apply those calculated speeds.
    runVelocity(speeds);
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
