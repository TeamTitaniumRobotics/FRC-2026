package org.teamtitanium.subsystems.genericroller;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.teamtitanium.subsystems.genericroller.GenericRoller.GenericRollerConstants;
import org.teamtitanium.utils.Constants;

public class GenericRollerIOSim extends GenericRollerIOTalonFX {
  private final DCMotorSim rollerSim;
  private final Notifier simNotifier;
  private double lastSimTime = 0.0;

  private GenericRollerConstants constants;

  public GenericRollerIOSim(GenericRollerConstants constants, DCMotor simMotor, double moi) {
    super(constants);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(simMotor, moi, constants.reduction()), simMotor);
    this.constants = constants;

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

              motorSimState.setRawRotorPosition(
                  rollerSim.getAngularPositionRotations() * constants.reduction());
              motorSimState.setRotorVelocity(
                  rollerSim.getAngularVelocityRPM() / 60.0 * constants.reduction());
            });
    simNotifier.startPeriodic(Constants.simLoopPeriodSecs);
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    inputs.motorConnected = true;

    super.updateInputs(inputs);
  }
}
