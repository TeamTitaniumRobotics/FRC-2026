package org.teamtitanium.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Gains;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public boolean leftMotorConnected = false;
    public double leftVelocityRps = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public boolean rightMotorConnected = false;
    public double rightVelocityRps = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Updates the inputs for the flywheel. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets the flywheel to the given velocity in rotations per second. */
  public default void setVelocity(double velocityRps) {}

  /** Sets the flywheel motors to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the flywheel motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Sets the brake mode for the flywheel motors. */
  public default void setBrakeMode(boolean enabled) {}

  /** Stops the flywheel motors. */
  public default void stop() {}
}
