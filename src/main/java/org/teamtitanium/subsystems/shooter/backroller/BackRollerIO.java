package org.teamtitanium.subsystems.shooter.backroller;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface BackRollerIO {
  @AutoLog
  public class BackRollerIOInputs {
    public boolean motorConnected = false;

    public double velocityRps = 0.0;
    public double velocitySetpoint = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the inputs for the back roller. */
  public default void updateInputs(BackRollerIOInputs inputs) {}

  /** Sets the back roller to the given velocity in rotations per second. */
  public default void setVelocity(double velocityRps) {}

  /** Sets the back roller motor to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the back roller motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Sets the constraints for the back roller motor. */
  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the back roller motor. */
  public default void setBrakeMode(boolean enabled) {}
}
