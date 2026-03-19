package org.teamtitanium.subsystems.genericroller;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.signals.RobotEnableValue;
import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface GenericRollerIO {
  @AutoLog
  public abstract class GenericRollerIOInputs {
    public boolean motorConnected = false;
    public double positionRots = 0.0;
    public double velocityRps = 0.0;
    public double velocitySetpoint = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;

    public DeviceEnableValue motorEnabled = DeviceEnableValue.Disabled;
    public RobotEnableValue robotEnabled = RobotEnableValue.Disabled;
    public ControlModeValue controlMode = ControlModeValue.DisabledOutput;
  }

  public default void updateInputs(GenericRollerIOInputs inputs) {}

  public default void setVelocity(double velocityRps) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default void setGains(Gains gains) {}

  public default void setConstraints(Constraints constraints) {}
}
