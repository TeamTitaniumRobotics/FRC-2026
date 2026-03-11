package org.teamtitanium.subsystems.intake.rack;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface IntakeRackIO {
  @AutoLog
  public class IntakeRackIOInputs {
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

  /** Updates the rack inputs. */
  public default void updateInputs(IntakeRackIOInputs inputs) {}

  /** Sets the rack to the given motor rotations via Motion Magic. */
  public default void setPosition(double positionRots) {}

  public default void setPosition(double positionRots, Constraints constraints) {}

  /** Runs the rack open loop at the requested voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the PID/FF constants used by the rack motor. */
  public default void setGains(Gains gains) {}

  /** Updates the Motion Magic constraints. */
  public default void setConstraints(Constraints constraints) {}

  /** Enables or disables brake mode. */
  public default void setBrakeMode(boolean enabled) {}

  /** Forces the internal motor encoder to a given rotation value. */
  public default void setMotorPosition(double positionRots) {}

  public default void stop() {}
}
