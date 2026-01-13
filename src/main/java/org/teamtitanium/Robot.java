// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.teamtitanium;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.extern.java.Log;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.teamtitanium.subsystems.Leds;
import org.teamtitanium.utils.CanivoreReader;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.PhoenixUtil;
import org.teamtitanium.utils.TunerConstants;
import org.teamtitanium.utils.VirtualSubsystem;
import org.teamtitanium.utils.Constants.Mode;

import com.ctre.phoenix6.SignalLogger;

public class Robot extends LoggedRobot {
  private static final double loopOverrunWarningTimeout = 0.02;
  private static final double canErrorTimeThreshold = 0.5;
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltageThreshold = 11.8;
  private static final double lowBatteryDisabledTimeThreshold = 2.0;
  private static final double lowBatteryMinCycleCount = 10.0;
  private static int lowBatteryCycleCount = 0;

  private Command autonomousCommand;

  private double autoStartTime = 0.0;
  private boolean autoMessagePrinted = false;
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private final Timer disabledTimer = new Timer();
  private final CanivoreReader canivoreReader = new CanivoreReader(TunerConstants.kCANBus);

  private final Alert canErrorAlert = new Alert("CAN Bus Error Detected", Alert.AlertType.kError);
  private final Alert canivoreErrorAlert = new Alert("Canivore CAN Bus Error Detected", Alert.AlertType.kError);
  private final Alert lowBatteryAlert = new Alert("Low Battery Voltage Detected", Alert.AlertType.kWarning);
  private final Alert initializationAlert = new Alert("Please wait to enable, robot is initializing", Alert.AlertType.kWarning);

  public Robot() {
    Leds.getInstance(); // Initialize LED subsystem early

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
        Logger.addDataReceiver(new RLOGServer());
        break;
      // Set up for simulation
      case SIM:
        Logger.addDataReceiver(new RLOGServer());
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

    // Disable CTRE Phoenix Pro auto logging to reduce overhead
    SignalLogger.enableAutoLogging(false);

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
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
      String commandName = command.getName();
      int count = commandCounts.getOrDefault(commandName, 0) + (active ? 1 : -1);
      commandCounts.put(commandName, count);
      Logger.recordOutput("CommandsUnique/" + commandName + "_" + Integer.toHexString(command.hashCode()), active);
      Logger.recordOutput("CommandsAll/" + commandName, count > 0);
    };
    CommandScheduler.getInstance().onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    RobotController.setBrownoutVoltage(6.0);

    if (Constants.getMode() == Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }
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
    LoggedTracer.record("Commands");

    if (autonomousCommand != null) {
      if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf("*** Autonomous finished in %.2f seconds ***%n", Timer.getTimestamp() - autoStartTime);
        }
        else {
          System.out.printf("*** Autonomous canceled in %.2f seconds ***%n", Timer.getTimestamp() - autoStartTime);
        }
        autoMessagePrinted = true;
      }
    }

    updateAlerts();
    updateDashboardOuputs();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = Commands.none(); // TODO: Add autonomous command here

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  private void updateAlerts() {

  }

  private void updateDashboardOuputs() {

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
