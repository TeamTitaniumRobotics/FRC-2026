package org.teamtitanium.subsystems.genericroller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.teamtitanium.subsystems.genericroller.GenericRoller.GenericRollerConstants;

public class GenericRollerIOSim extends GenericRollerIOTalonFX {
  private final DCMotorSim rollerSim;
  private GenericRollerConstants constants;

  public GenericRollerIOSim(GenericRollerConstants constants, DCMotor simMotor, double moi) {
    super(constants);
    rollerSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(simMotor, moi, constants.reduction()), simMotor);
    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    inputs.motorConnected = true;

    var motorSimState = rollerMotor.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    rollerSim.setInputVoltage(motorSimState.getMotorVoltage());
    rollerSim.update(0.02);

    motorSimState.setRawRotorPosition(
        rollerSim.getAngularPositionRotations() * constants.reduction());
    motorSimState.setRotorVelocity(
        rollerSim.getAngularVelocityRPM() / 60.0 * constants.reduction());

    super.updateInputs(inputs);
  }
}
