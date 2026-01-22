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
  }

  public void updateInputs(TurretIOInputs inputs);

  public default void setPosition(double positionRots) {}

  public default void setVoltage(double volts) {}

  public default void setGains(Gains gains) {}

  public default void setConstraints(Constraints constraints) {}

  public default void setBrakeMode(boolean enabled) {}

  public default void setMotorPosition(double positionRots) {}
}
