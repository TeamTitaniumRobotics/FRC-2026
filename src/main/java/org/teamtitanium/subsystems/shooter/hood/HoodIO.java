package org.teamtitanium.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface HoodIO {
  @AutoLog
  public class HoodIOInputs {
    public boolean motorConnected = false;
    public double positionRots = 0.0;
    public double setpointRots = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the inputs for the hood. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Sets the hood to the given position in rotations using Motion Magic. */
  public default void setPosition(double positionRots) {}

  /** Sets the hood motor to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  public default void stopMotor() {}

  /** Updates the hood motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Updates the Motion Magic constraints for the hood. */
  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the hood motor. */
  public default void setBrakeMode(boolean enabled) {}

  /** Sets the hood motor encoder position to the given value in rotations. */
  public default void setMotorPosition(double positionRots) {}
}
