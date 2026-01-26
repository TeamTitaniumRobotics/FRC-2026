package org.teamtitanium.subsystems.shooter.turret;

import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim extends TurretIOTalonFX {
  private final DCMotorSim turretSim;

  private double appliedVolts = 0.0;

  public TurretIOSim() {
    super();

    // Create simulation model
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURRET_GEARBOX, TURRET_MOMENT_OF_INERTIA, TURRET_GEAR_RATIO),
            TURRET_GEARBOX);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Update simulation
    turretSim.setInputVoltage(appliedVolts);
    turretSim.update(0.02); // 20ms loop time

    // Get simulated values
    inputs.motorConnected = true;
    inputs.positionRots = turretSim.getAngularPositionRotations();
    inputs.velocityRps = turretSim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = turretSim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = turretSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0; // Assume constant temperature in sim

    // Simulate CANcoders
    inputs.cancoder1Connected = true;
    inputs.cancoder1PositionRots = inputs.positionRots * CANCODER_1_RATIO;

    inputs.cancoder2Connected = true;
    inputs.cancoder2PositionRots = inputs.positionRots * CANCODER_2_RATIO;

    // Update the real motor position for Motion Magic control
    turretMotor.getSimState().setRawRotorPosition(inputs.positionRots * TURRET_GEAR_RATIO);
    turretMotor.getSimState().setRotorVelocity(inputs.velocityRps * TURRET_GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    super.setVoltage(volts);
    appliedVolts = volts;
  }

  @Override
  public void setPosition(double positionRots) {
    super.setPosition(positionRots);
    // Get the voltage output from the TalonFX controller
    appliedVolts = turretMotor.getSimState().getMotorVoltage();
  }

  @Override
  public void stop() {
    super.stop();
    appliedVolts = 0.0;
  }
}
