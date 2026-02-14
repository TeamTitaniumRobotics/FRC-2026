package org.teamtitanium.subsystems.intake.rack;

import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.RACK_GEARBOX;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.RACK_GEAR_RATIO;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.RACK_MOMENT_OF_INERTIA;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants;

public class IntakeRackIOSim extends IntakeRackIOTalonFX {
  private final DCMotorSim rackSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(RACK_GEARBOX, RACK_MOMENT_OF_INERTIA, RACK_GEAR_RATIO),
          RACK_GEARBOX);

  @Override
  public void updateInputs(IntakeRackIOInputs inputs) {
    inputs.motorConnected = true;

    var motorSimState = rackMotor.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    rackSim.setInputVoltage(motorSimState.getMotorVoltage());
    rackSim.update(Constants.loopPeriodSecs);

    motorSimState.setRawRotorPosition(rackSim.getAngularPositionRotations() * RACK_GEAR_RATIO);
    motorSimState.setRotorVelocity(rackSim.getAngularVelocityRPM() / 60.0 * RACK_GEAR_RATIO);

    super.updateInputs(inputs);

    Logger.recordOutput(
        "Intake/Rack/Sim/Position", new Pose3d(inputs.positionMeters, 0.0, 0.0, Rotation3d.kZero));
  }
}
