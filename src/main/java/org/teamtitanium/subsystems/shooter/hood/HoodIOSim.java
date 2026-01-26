package org.teamtitanium.subsystems.shooter.hood;

import static org.teamtitanium.subsystems.shooter.hood.HoodConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim extends HoodIOTalonFX {
  private final DCMotorSim hoodSim;

  private double appliedVolts = 0.0;

  public HoodIOSim() {
    super();

    // Create simulation model
    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HOOD_GEARBOX, HOOD_MOMENT_OF_INERTIA, HOOD_GEAR_RATIO),
            HOOD_GEARBOX);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Update simulation
    hoodSim.setInputVoltage(appliedVolts);
    hoodSim.update(0.02); // 20ms loop time

    // Get simulated values
    inputs.motorConnected = true;
    inputs.positionRots = hoodSim.getAngularPositionRotations();
    inputs.velocityRps = hoodSim.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = hoodSim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = hoodSim.getCurrentDrawAmps();
    inputs.tempCelsius = 25.0; // Assume constant temperature in sim

    // Simulate CANcoder
    inputs.cancoderConnected = true;
    inputs.cancoderPositionRots = inputs.positionRots;

    // Update the real motor position for Motion Magic control
    hoodMotor.getSimState().setRawRotorPosition(inputs.positionRots * HOOD_GEAR_RATIO);
    hoodMotor.getSimState().setRotorVelocity(inputs.velocityRps * HOOD_GEAR_RATIO);
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
    appliedVolts = hoodMotor.getSimState().getMotorVoltage();
  }

  @Override
  public void stop() {
    super.stop();
    appliedVolts = 0.0;
  }
}
