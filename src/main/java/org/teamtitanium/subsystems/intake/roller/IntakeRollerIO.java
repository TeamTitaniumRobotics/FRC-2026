package org.teamtitanium.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface IntakeRollerIO {
  @AutoLog
  public class IntakeRollerIOInputs {
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

  /** Updates the inputs for the intake roller. */
  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  /** Sets the intake roller to the given velocity in rotations per second. */
  public default void setVelocity(double velocityRps) {}

  /** Sets the intake roller motors to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the intake roller motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Sets the motion constraints for the intake roller. */
  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the intake roller motors. */
  public default void setBrakeMode(boolean enabled) {}

  public default void stop() {}
}
