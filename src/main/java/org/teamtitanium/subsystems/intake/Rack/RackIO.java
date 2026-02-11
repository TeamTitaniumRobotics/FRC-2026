package org.teamtitanium.subsystems.intake.Rack;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface RackIO {
  @AutoLog
  public class RackIOInputs {
    public boolean motorConnected = false;
    public double positionRots = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the inputs for the rack and pinions. */
  public default void updateInputs(RackIOInputs inputs) {}

  /** Sets the rack and pinions to the given position in rotations using Motion Magic. */
  public default void setPosition(double positionRots) {}

  /** Sets the rack and pinions motor to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the rack and pinions motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Updates the Motion Magic constraints for the rack and pinions. */
  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the rack and pinions motor. */
  public default void setBrakeMode(boolean enabled) {}

  /** Sets the rack and pinions motor encoder position to the given value in rotations. */
  public default void setMotorPosition(double positionRots) {}
}
