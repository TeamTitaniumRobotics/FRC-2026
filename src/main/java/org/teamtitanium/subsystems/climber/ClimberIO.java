package org.teamtitanium.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public boolean motorConnected = false;
    public double positionRots = 0.0;
    public double positionMeters = 0.0;
    public double velocityRps = 0.0;
    public double velocityMps = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public double setpointRots = 0.0;
    public double setpointMeters = 0.0;
  }

  /** Updates the climber inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Sets the climber to the given motor rotations. */
  public default void setPosition(double positionRots, int slotId) {}

  /** Runs the climber open loop at the requested current. */
  public default void setCurrent(double amps) {}

  /** Enables or disables brake mode. */
  public default void setBrakeMode(boolean enabled) {}

  /** Forces the internal motor encoder to a given rotation value. */
  public default void setMotorPosition(double positionRots) {}

  /** Sets the PID gains for the climber. */
  public default void setGains(Gains gains, int slotId) {}

  /** Sets the motion constraints for the climber. */
  public default void setConstraints(Constraints constraints) {}

  /** Stops the climber. */
  public default void stop() {}
}
