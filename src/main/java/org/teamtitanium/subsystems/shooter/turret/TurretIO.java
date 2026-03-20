package org.teamtitanium.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface TurretIO {
  @AutoLog
  public class TurretIOInputs {
    public boolean motorConnected = false;
    public double positionRots = 0.0;
    public double velocityRps = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public double setpointRots = 0.0;

    public boolean cancoder1Connected = false;
    public double cancoder1PositionRots = 0.0;

    public boolean cancoder2Connected = false;
    public double cancoder2PositionRots = 0.0;
  }

  /** Updates the inputs for the turret. */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Sets the turret to the given position in rotations. */
  public default void setPosition(double positionRots) {}

  /** Sets the turret motor to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the turret motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Updates the Motion Magic constraints for the turret. */
  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the turret motor. */
  public default void setBrakeMode(boolean enabled) {}

  /** Sets the turret motor encoder position to the given value in rotations. */
  public default void setMotorPosition(double positionRots) {}

  public default void stop() {}
}
