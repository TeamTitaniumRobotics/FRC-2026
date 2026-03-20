// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.teamtitanium;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.teamtitanium.autos.*;
import org.teamtitanium.commands.DriveCommands;
import org.teamtitanium.subsystems.Superstructure;
import org.teamtitanium.subsystems.feeder.Feeder;
import org.teamtitanium.subsystems.genericroller.GenericRollerIO;
import org.teamtitanium.subsystems.genericroller.GenericRollerIOSim;
import org.teamtitanium.subsystems.genericroller.GenericRollerIOTalonFX;
import org.teamtitanium.subsystems.intake.Intake;
import org.teamtitanium.subsystems.intake.IntakeConstants;
import org.teamtitanium.subsystems.intake.rack.IntakeRack;
import org.teamtitanium.subsystems.intake.rack.IntakeRackIO;
import org.teamtitanium.subsystems.intake.rack.IntakeRackIOSim;
import org.teamtitanium.subsystems.intake.rack.IntakeRackIOTalonFX;
import org.teamtitanium.subsystems.intake.roller.IntakeRoller;
import org.teamtitanium.subsystems.leds.LEDs;
import org.teamtitanium.subsystems.leds.LEDsIO;
import org.teamtitanium.subsystems.leds.LEDsIOReal;
import org.teamtitanium.subsystems.shooter.Shooter;
import org.teamtitanium.subsystems.shooter.ShotCalculator;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel;
import org.teamtitanium.subsystems.shooter.flywheel.FlywheelIO;
import org.teamtitanium.subsystems.shooter.flywheel.FlywheelIOSim;
import org.teamtitanium.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import org.teamtitanium.subsystems.shooter.hood.Hood;
import org.teamtitanium.subsystems.shooter.hood.HoodIO;
import org.teamtitanium.subsystems.shooter.hood.HoodIOSim;
import org.teamtitanium.subsystems.shooter.hood.HoodIOTalonFX;
import org.teamtitanium.subsystems.shooter.turret.Turret;
import org.teamtitanium.subsystems.shooter.turret.TurretIO;
import org.teamtitanium.subsystems.shooter.turret.TurretIOSim;
import org.teamtitanium.subsystems.shooter.turret.TurretIOTalonFX;
import org.teamtitanium.subsystems.spindexer.Spindexer;
import org.teamtitanium.subsystems.swerve.GyroIO;
import org.teamtitanium.subsystems.swerve.GyroIOPigeon2;
import org.teamtitanium.subsystems.swerve.Swerve;
import org.teamtitanium.subsystems.swerve.SwerveModuleIO;
import org.teamtitanium.subsystems.swerve.SwerveModuleIOSim;
import org.teamtitanium.subsystems.swerve.SwerveModuleIOTalonFX;
import org.teamtitanium.subsystems.vision.Vision;
import org.teamtitanium.subsystems.vision.VisionConstants;
import org.teamtitanium.subsystems.vision.VisionIO;
import org.teamtitanium.subsystems.vision.VisionIOPhoton;
import org.teamtitanium.subsystems.vision.VisionIOSim;
import org.teamtitanium.utils.CanivoreReader;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Mode;
import org.teamtitanium.utils.HubTracker;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.NTClientLogger;
import org.teamtitanium.utils.PhoenixUtil;
import org.teamtitanium.utils.TunerConstants;
import org.teamtitanium.utils.virtualsubsystem.VirtualSubsystem;

public class Robot extends LoggedRobot {
  private static final double loopOverrunWarningTimeout = 0.02;
  private static final double canErrorTimeThreshold = 0.5;
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltageThreshold = 11.8;
  private static final double lowBatteryDisabledTimeThreshold = 2.0;
  private static final double lowBatteryMinCycleCount = 10.0;
  private static int lowBatteryCycleCount = 0;

  private Command autonomousCommand;

  private final Swerve swerve;
  private final Shooter shooter;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final Feeder feeder;
  private final Spindexer spindexer;
  private final Intake intake;
  private final IntakeRack intakeRack;
  private final IntakeRoller intakeRoller;
  private final Superstructure superstructure;
  private final LEDs leds;
  private final Vision vision;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  private final ShotCalculator shotCalculator = ShotCalculator.getInstance();

  private double autoStartTime = 0.0;
  private boolean autoMessagePrinted = false;
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private final CanivoreReader canivoreReader = new CanivoreReader(TunerConstants.kCANBus);

  private static boolean coastOverride = false;

  @Getter
  @AutoLogOutput(key = "Dashboard/Commands/CoastOverride")
  private static Trigger coastOverrideTrigger =
      new Trigger(() -> coastOverride && DriverStation.isDisabled());

  private final Alert canErrorAlert = new Alert("CAN Bus Error Detected", Alert.AlertType.kError);
  private final Alert canivoreErrorAlert =
      new Alert("Canivore CAN Bus Error Detected", Alert.AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert("Low Battery Voltage Detected", Alert.AlertType.kWarning);
  private final Alert initializationAlert =
      new Alert("Please wait to enable, robot is initializing", Alert.AlertType.kWarning);

  private final AutoChooser autoChooser = new AutoChooser();
  private final AutoRoutines autoRoutines;

  private final Field2d field2d = new Field2d();

  public Robot() {
    // Set up logger
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Configure data receivers and replay sources
    switch (Constants.getMode()) {
        // Set up for real robot
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
        // Set up for simulation
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
        // Set up for replaying logs
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start the logger
    Logger.start();

    // Disable CTRE Phoenix Pro auto logging to reduce overhead (enable only in tuning mode)
    if (Constants.tuningMode) {
      SignalLogger.enableAutoLogging(true);
    } else {
      SignalLogger.enableAutoLogging(false);
    }

    // Adjust loop timing overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(loopOverrunWarningTimeout);

    // Silence joystick connection warnings, will use Alerts instead
    DriverStation.silenceJoystickConnectionWarning(true);

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String commandName = command.getName();
          int count = commandCounts.getOrDefault(commandName, 0) + (active ? 1 : -1);
          commandCounts.put(commandName, count);
          Logger.recordOutput(
              "CommandsUnique/" + commandName + "_" + Integer.toHexString(command.hashCode()),
              active);
          Logger.recordOutput("CommandsAll/" + commandName, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    RobotController.setBrownoutVoltage(6.0);

    if (Constants.getMode() == Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    switch (Constants.getMode()) {
      case REAL -> {
        swerve =
            new Swerve(
                new GyroIOPigeon2(),
                new SwerveModuleIOTalonFX(TunerConstants.FrontLeft),
                new SwerveModuleIOTalonFX(TunerConstants.FrontRight),
                new SwerveModuleIOTalonFX(TunerConstants.BackLeft),
                new SwerveModuleIOTalonFX(TunerConstants.BackRight));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        turret = new Turret(new TurretIOTalonFX());
        feeder = new Feeder(new GenericRollerIOTalonFX(Feeder.CONSTANTS));
        spindexer = new Spindexer(new GenericRollerIOTalonFX(Spindexer.CONSTANTS));
        intakeRack = new IntakeRack(new IntakeRackIOTalonFX());
        intakeRoller =
            new IntakeRoller(new GenericRollerIOTalonFX(IntakeConstants.RollerConstants.CONSTANTS));
        leds = new LEDs(new LEDsIOReal());
        vision =
            new Vision(
                new VisionIOPhoton(
                    VisionConstants.frontCameraName, VisionConstants.frontCameraPose),
                new VisionIOPhoton(VisionConstants.backCameraName, VisionConstants.backCameraPose),
                new VisionIOPhoton(VisionConstants.leftCameraName, VisionConstants.leftCameraPose),
                new VisionIOPhoton(VisionConstants.frCameraName, VisionConstants.frCameraPose));
      }
      case SIM -> {
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIOSim(TunerConstants.FrontLeft),
                new SwerveModuleIOSim(TunerConstants.FrontRight),
                new SwerveModuleIOSim(TunerConstants.BackLeft),
                new SwerveModuleIOSim(TunerConstants.BackRight));
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());
        turret = new Turret(new TurretIOSim());
        feeder =
            new Feeder(
                new GenericRollerIOSim(
                    Feeder.CONSTANTS, Feeder.FEEDER_MOTOR_GEARBOX, Feeder.FEEDER_MOI));
        spindexer =
            new Spindexer(
                new GenericRollerIOSim(
                    Spindexer.CONSTANTS,
                    Spindexer.SPINDEXER_MOTOR_GEARBOX,
                    Spindexer.SPINDEXER_MOI));
        intakeRack = new IntakeRack(new IntakeRackIOSim());
        intakeRoller =
            new IntakeRoller(
                new GenericRollerIOSim(
                    IntakeConstants.RollerConstants.CONSTANTS,
                    IntakeConstants.RollerConstants.ROLLER_MOTOR_GEARBOX,
                    IntakeConstants.RollerConstants.ROLLER_MOI));
        leds = new LEDs(new LEDsIO() {});
        vision =
            new Vision(
                new VisionIOSim(
                    VisionConstants.frontCameraName,
                    VisionConstants.frontCameraPose,
                    () -> RobotState.getInstance().getEstimatedPose()),
                new VisionIOSim(
                    VisionConstants.backCameraName,
                    VisionConstants.backCameraPose,
                    () -> RobotState.getInstance().getEstimatedPose()),
                new VisionIOSim(
                    VisionConstants.leftCameraName,
                    VisionConstants.leftCameraPose,
                    () -> RobotState.getInstance().getEstimatedPose()));
      }
      default -> {
        swerve =
            new Swerve(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIO() {});
        feeder = new Feeder(new GenericRollerIO() {});
        spindexer = new Spindexer(new GenericRollerIO() {});
        intakeRack = new IntakeRack(new IntakeRackIO() {});
        intakeRoller = new IntakeRoller(new GenericRollerIO() {});
        leds = new LEDs(new LEDsIO() {});
        vision = new Vision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
      }
    }

    intake = new Intake(intakeRack, intakeRoller);
    shooter = new Shooter(flywheel, hood, turret);
    superstructure = new Superstructure(shooter, feeder, spindexer, intake, driver);

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> Logger.recordOutput("Autos/CurrentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> Logger.recordOutput("Autos/TargetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        (poses) ->
            Logger.recordOutput("Autos/ActivePath", poses.toArray(new Pose2d[poses.size()])));

    autoRoutines = new AutoRoutines(swerve);
    autoChooser.addCmd("Right Outpost", autoRoutines::getRightOutpostAuto);
    autoChooser.addCmd("Left Double Pass", autoRoutines::leftDoublePass);
    autoChooser.addCmd("Straight Test", autoRoutines::straightTuningAuto);

    autoChooser.addCmd(
        "Swerve FF Characterization", () -> DriveCommands.feedforwardCharacterization(swerve));
    autoChooser.addCmd(
        "Swerve Wheel Radius Characterization",
        () -> DriveCommands.wheelRadiusCharacterization(swerve));

    SmartDashboard.putData("Autos/AutoChooser", autoChooser);

    RobotModeTriggers.autonomous()
        .onTrue(Commands.runOnce(() -> autonomousCommand = autoChooser.selectedCommand()));
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    configureButtonBindings();
    configureDashboard();
  }

  @Override
  public void robotPeriodic() {
    // Refresh Phoenix devices
    LoggedTracer.reset();
    PhoenixUtil.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    // Run virtual subsystems (e.g. LEDs, Vision, etc.)
    VirtualSubsystem.periodicAll();

    // Run the CommandScheduler
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Robot/Commands");

    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Autonomous finished in %.2f seconds ***%n",
              Timer.getTimestamp() - autoStartTime);
        } else {
          System.out.printf(
              "*** Autonomous canceled in %.2f seconds ***%n",
              Timer.getTimestamp() - autoStartTime);
        }
        autoMessagePrinted = true;
      }
    }

    updateAlerts();
    updateDashboardOuputs();

    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    if (Constants.getMode() == Constants.Mode.REAL) {
      var canivoreStatus = canivoreReader.getStatus();
      if (canivoreStatus.isPresent()) {
        Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.get().Status.getName());
        Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.get().BusUtilization);
        Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.get().BusOffCount);
        Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.get().TxFullCount);
        Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.get().REC);
        Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.get().TEC);
        if (!canivoreStatus.get().Status.isOK()
            || canStatus.transmitErrorCount > 0
            || canStatus.receiveErrorCount > 0) {
          canivoreErrorTimer.restart();
        }
      } else {
        Logger.recordOutput("CANivoreStatus/Status", "CANivore Status Not Present");
      }

      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
              && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }

    // Log NetworkTables data
    NTClientLogger.log();

    // Low Battery Alert
    lowBatteryCycleCount++;
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltageThreshold
        && disabledTimer.hasElapsed(lowBatteryDisabledTimeThreshold)
        && lowBatteryCycleCount >= lowBatteryMinCycleCount) {
      lowBatteryAlert.set(true);
      LEDs.getInstance().setLowBatteryAlert(true);
    } else {
      lowBatteryCycleCount = 0;
      lowBatteryAlert.set(false);
      LEDs.getInstance().setLowBatteryAlert(false);
    }

    // Initialization Alert
    initializationAlert.set(isInitializing());

    field2d.setRobotPose(RobotState.getInstance().getEstimatedPose());

    // Log hub state
    Logger.recordOutput("HubTracker/Official", HubTracker.getOfficialShiftInfo());
    Logger.recordOutput("HubTracker/Offset", HubTracker.getOffsetShiftInfo());

    // Log shot parameters
    Logger.recordOutput("ShotCalculator/Parameters", shotCalculator.getParameters());
    shotCalculator.resetShotParameters();

    LoggedTracer.record("Robot/Periodic");
  }

  private void configureButtonBindings() {
    swerve.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false));

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .setEstimatedPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    Rotation2d.kZero)))
                .ignoringDisable(true));

    driver
        .rightBumper()
        .whileTrue(
            DriveCommands.trenchDrive(
                swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX()));

    superstructure
        .getIntakeDeployed()
        .onTrue(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0.01)))
        .onFalse(Commands.runOnce(() -> driver.setRumble(RumbleType.kBothRumble, 0.0)));

    driver.back().whileTrue(hood.zeroHood()); // TODO: Disable roller while zeroing rack
    // driver.back().whileTrue(Commands.parallel(hood.zeroHood(), intakeRack.zeroIntake()));
    driver.start().whileTrue(intake.zeroIntake());

    driver.leftStick().whileTrue(Commands.runOnce(() -> swerve.stopWithX(), swerve));

    driver
        .povRight()
        .whileTrue(spindexer.setVoltage(() -> -6.0).alongWith(feeder.setVoltage(() -> -6.0)));

    copilot
        .rightBumper()
        .onTrue(Commands.runOnce(() -> shotCalculator.incrementFlywheelOffset(50.0)));
    copilot
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> shotCalculator.incrementFlywheelOffset(-50.0)));
    copilot.a().onTrue(Commands.runOnce(() -> shotCalculator.setFlywheelOffset(0.0)));

    RobotModeTriggers.teleop()
        .or(RobotModeTriggers.autonomous())
        .or(RobotModeTriggers.disabled())
        .onTrue(Commands.runOnce(HubTracker::initialize).ignoringDisable(true));
  }

  private void updateAlerts() {}

  private void configureDashboard() {
    SmartDashboard.putData("Dashboard/Field2d", field2d);

    SmartDashboard.putData(
        "Dashboard/Commands/ZeroTurret",
        Commands.runOnce(() -> turret.zeroTurretCRT(), turret).ignoringDisable(true));
    SmartDashboard.putData(
        "Dashboard/Commands/DisableTurret",
        Commands.runOnce(
                () -> {
                  shooter.setTurretDisabled(!shooter.isTurretDisabled());
                  leds.setEStopped(shooter.isTurretDisabled());
                })
            .ignoringDisable(true)
            .withName("Turret Disable"));

    SmartDashboard.putData(
        "Dashboard/Commands/Coast",
        Commands.runOnce(
                () -> {
                  if (DriverStation.isDisabled()) {
                    coastOverride = !coastOverride;
                    leds.setCoastOverride(coastOverride);
                  }
                })
            .ignoringDisable(true)
            .withName("Coast Override"));

    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
                      leds.setCoastOverride(coastOverride);
                    })
                .ignoringDisable(true));
  }

  private void updateDashboardOuputs() {
    SmartDashboard.putNumber("Dashboard/MatchTime", DriverStation.getMatchTime());

    SmartDashboard.putString(
        "Dashboard/HubTracker/RemainingShiftTime",
        String.format("%.1f", Math.max(HubTracker.getOffsetShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean(
        "Dashboard/HubTracker/HubActive", HubTracker.getOffsetShiftInfo().active());
    SmartDashboard.putString(
        "Dashboard/HubTracker/ShiftState",
        HubTracker.getOffsetShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Dashboard/HubTracker/ActiveFirst",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubTracker.getFirstActiveAlliance());
  }

  public static boolean isInitializing() {
    return Timer.getTimestamp() < 45.0;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Logger.recordOutput("Autos/SelectedAuto", autoChooser.selectedCommand().getName());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoStartTime = Timer.getTimestamp();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationPeriodic() {
    VirtualSubsystem.simulationPeriodicAll();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
