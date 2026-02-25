package org.teamtitanium.subsystems.shooter.flywheel;

import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.teamtitanium.utils.Constants;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private final FlywheelSim flywheelSim;
  private final Notifier simNotifier;
  private double lastSimTime = 0.0;

  public FlywheelIOSim() {
    super();

    // Create simulation model (two motors combined)
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                FLYWHEEL_GEARBOX, FLYWHEEL_MOMENT_OF_INERTIA, FLYWHEEL_GEAR_RATIO),
            FLYWHEEL_GEARBOX);

    var motorSimState = leftMotor.getSimState();
    motorSimState.setMotorType(MotorType.KrakenX60);

    lastSimTime = Utils.getCurrentTimeSeconds();
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              flywheelSim.setInputVoltage(motorSimState.getMotorVoltage());
              flywheelSim.update(deltaTime);

              // motorSimState.setRawRotorPosition(
              //     flywheelSim.getAngularPositionRotations() * constants.reduction());
              motorSimState.setRotorVelocity(
                  flywheelSim.getAngularVelocityRPM()
                      / 60.0
                      * FlywheelConstants.FLYWHEEL_GEAR_RATIO);
            });
    simNotifier.startPeriodic(Constants.simLoopPeriodSecs);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;

    super.updateInputs(inputs);
  }
}
