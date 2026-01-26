package org.teamtitanium.subsystems.shooter.flywheel;

import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private final DCMotorSim flywheelSim;

  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    super();

    // Create simulation model (two motors combined)
    flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FLYWHEEL_GEARBOX, FLYWHEEL_MOMENT_OF_INERTIA, FLYWHEEL_GEAR_RATIO),
            FLYWHEEL_GEARBOX);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Update simulation
    flywheelSim.setInputVoltage(appliedVolts);
    flywheelSim.update(0.02); // 20ms loop time

    // Get simulated values (both motors see same velocity)
    double velocityRps = flywheelSim.getAngularVelocityRPM() / 60.0;
    double currentAmps = flywheelSim.getCurrentDrawAmps() / 2.0; // Split between two motors

    inputs.leftMotorConnected = true;
    inputs.leftVelocityRps = velocityRps;
    inputs.leftAppliedVolts = appliedVolts;
    inputs.leftSupplyCurrentAmps = currentAmps;
    inputs.leftTorqueCurrentAmps = currentAmps;
    inputs.leftTempCelsius = 25.0; // Assume constant temperature in sim

    inputs.rightMotorConnected = true;
    inputs.rightVelocityRps = velocityRps;
    inputs.rightAppliedVolts = appliedVolts;
    inputs.rightSupplyCurrentAmps = currentAmps;
    inputs.rightTorqueCurrentAmps = currentAmps;
    inputs.rightTempCelsius = 25.0;

    // Update the real motor velocities for velocity control
    leftMotor.getSimState().setRotorVelocity(velocityRps * FLYWHEEL_GEAR_RATIO);
    rightMotor.getSimState().setRotorVelocity(velocityRps * FLYWHEEL_GEAR_RATIO);
  }

  @Override
  public void setVoltage(double volts) {
    super.setVoltage(volts);
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double velocityRps) {
    super.setVelocity(velocityRps);
    // Get the voltage output from the TalonFX controller
    appliedVolts = leftMotor.getSimState().getMotorVoltage();
  }

  @Override
  public void stop() {
    super.stop();
    appliedVolts = 0.0;
  }
}
