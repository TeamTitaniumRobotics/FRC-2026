package org.teamtitanium.subsystems.intake.Roller;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;

public interface RollerBarIO {
    @AutoLog
    public class RollerIOInputs {
        public boolean motorConnected = false;
        public double velocityRps = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
  }

  /** Updates the inputs for the rollerbar. */
  public default void updateInputs(RollerIOInputs inputs) {}

  /** Sets the rollerbar motor to open loop control with the given voltage. */
  public default void setVoltage(double volts) {}

  /** Updates the rollerbar motor PID gains. */
  public default void setGains(Gains gains) {}

  /** Updates the Motion Magic constraints for the rollerbar. */
  public default void setConstraints(Constraints constraints) {}

  /** Sets the brake mode for the rollerbar motor. */
  public default void setBrakeMode(boolean enabled) {}

}
