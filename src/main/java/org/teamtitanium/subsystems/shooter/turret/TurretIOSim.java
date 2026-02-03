package org.teamtitanium.subsystems.shooter.turret;

import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.RobotState;

public class TurretIOSim extends TurretIOTalonFX {
  private final DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              TURRET_GEARBOX, TURRET_MOMENT_OF_INERTIA, TURRET_GEAR_RATIO),
          TURRET_GEARBOX);

  // private double appliedVolts = 0.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = true;

    var motorSimState = turretMotor.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update simulation
    turretSim.setInputVoltage(motorSimState.getMotorVoltage());
    turretSim.update(0.02); // 20ms loop time

    // Get simulated values
    // inputs.motorConnected = true;
    // inputs.positionRots = turretSim.getAngularPositionRotations();
    // inputs.velocityRps = turretSim.getAngularVelocityRPM() / 60.0;
    // inputs.appliedVolts = appliedVolts;
    // inputs.supplyCurrentAmps = turretSim.getCurrentDrawAmps();
    // inputs.torqueCurrentAmps = turretSim.getCurrentDrawAmps();
    // inputs.tempCelsius = 25.0; // Assume constant temperature in sim

    // inputs.setpointRots = super.targetSetpoint.getValueAsDouble();

    // Simulate CANcoders
    // inputs.cancoder1Connected = true;
    // inputs.cancoder1PositionRots = inputs.positionRots * CANCODER_1_RATIO;

    // inputs.cancoder2Connected = true;
    // inputs.cancoder2PositionRots = inputs.positionRots * CANCODER_2_RATIO;

    // Update the real motor position for Motion Magic control
    motorSimState.setRawRotorPosition(turretSim.getAngularPositionRotations() * TURRET_GEAR_RATIO);
    motorSimState.setRotorVelocity(turretSim.getAngularVelocityRPM() / 60.0 * TURRET_GEAR_RATIO);

    super.updateInputs(inputs);

    Logger.recordOutput(
        "Turret/Sim/TurretPosition",
        new Pose3d(RobotState.getInstance().getEstimatedPose())
            .plus(TURRET_TO_ROBOT)
            .plus(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 0.0, Units.rotationsToRadians(inputs.positionRots)))));
  }

  // @Override
  // public void setPosition(double positionRots) {
  //   super.setPosition(positionRots);
  //   // Get the voltage output from the TalonFX controller
  //   appliedVolts = turretMotor.getSimState().getMotorVoltage();
  // }

  // @Override
  // public void setVoltage(double volts) {
  //   super.setVoltage(volts);
  //   appliedVolts = volts;
  // }

  // @Override
  // public void setGains(Gains gains) {
  //   super.setGains(gains);
  // }

  // @Override
  // public void setConstraints(Constraints constraints) {
  //   super.setConstraints(constraints);
  // }
}
