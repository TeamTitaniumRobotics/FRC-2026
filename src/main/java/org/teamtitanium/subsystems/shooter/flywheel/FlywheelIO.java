package org.teamtitanium.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public boolean leftMotorConnected = false;
    public boolean rightMotorConnected = false;

    public double positionRots = 0.0;
    public double velocityRps = 0.0;
    public double velocitySetpoint = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  /** Updates the inputs for the flywheel. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets the flywheel to the given velocity in rotations per second. */
  public default void setVelocity(double velocityRps) {}

  /** Sets the flywheel motors to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the flywheel motor PID gains. */
  public default void setGains(Gains gains, int slotId) {}

  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the flywheel motors. */
  public default void setBrakeMode(boolean enabled) {}
}
