package org.teamtitanium.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDsIO {
  @AutoLog
  public class LEDsIOInputs {
    public byte[] buffer = new byte[0];
  }

  public default void updateInputs(LEDsIOInputs inputs) {}
}
