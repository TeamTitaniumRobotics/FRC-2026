package org.teamtitanium.subsystems.shooter.backroller;

import static org.teamtitanium.subsystems.shooter.backroller.BackRollerConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.teamtitanium.utils.Constants;

public class BackRollerIOSim extends BackRollerIOTalonFX {
  private final DCMotorSim rollerSim;
  private final Notifier simNotifier;
  private double lastSimTime = 0.0;

  public BackRollerIOSim() {
    super();

    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX44(1), BACK_ROLLER_MOMENT_OF_INERTIA, BACK_ROLLER_GEAR_RATIO),
            DCMotor.getKrakenX44(1));

    var motorSimState = rollerMotor.getSimState();
    motorSimState.setMotorType(MotorType.KrakenX44);

    lastSimTime = Utils.getCurrentTimeSeconds();
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              rollerSim.setInputVoltage(motorSimState.getMotorVoltage());
              rollerSim.update(deltaTime);

              motorSimState.setRotorVelocity(
                  rollerSim.getAngularVelocityRPM() / 60.0 * BACK_ROLLER_GEAR_RATIO);
            });
    simNotifier.startPeriodic(Constants.simLoopPeriodSecs);
  }

  @Override
  public void updateInputs(BackRollerIOInputs inputs) {
    inputs.motorConnected = true;

    super.updateInputs(inputs);
  }
}
