package org.teamtitanium.subsystems.intake.roller;

import static org.teamtitanium.subsystems.intake.IntakeConstants.RollerConstants.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.teamtitanium.utils.Constants;

public class IntakeRollerIOSim extends IntakeRollerIOTalonFX {
  private final DCMotorSim rollerSim;
  private final Notifier simNotifier;
  private double lastSimTime = 0.0;

  public IntakeRollerIOSim() {
    super();

    // Create simulation model (two motors combined)
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(ROLLER_MOTOR_GEARBOX, ROLLER_MOI, ROLLER_GEAR_RATIO),
            ROLLER_MOTOR_GEARBOX);

    var motorSimState = leftMotor.getSimState();
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
                  rollerSim.getAngularVelocityRPM() / 60.0 * ROLLER_GEAR_RATIO);
            });
    simNotifier.startPeriodic(Constants.simLoopPeriodSecs);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.leftMotorConnected = true;
    inputs.rightMotorConnected = true;

    super.updateInputs(inputs);
  }
}
