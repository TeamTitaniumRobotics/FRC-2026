package org.teamtitanium.subsystems.leds;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;

public class LEDsIOReal implements LEDsIO {
  private final int handle;

  // private final AddressableLED leds = new AddressableLED(LEDConstants.PORT);
  // private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.LENGTH);

  public LEDsIOReal() {
    int pwmHandle = PWMJNI.initializePWMPort(HAL.getPort((byte) LEDConstants.PORT));
    handle = AddressableLEDJNI.initialize(pwmHandle);
    AddressableLEDJNI.setLength(handle, LEDConstants.LENGTH);
    AddressableLEDJNI.start(handle);

    // leds.setColorOrder(ColorOrder.kRGB);
    // leds.setLength(LEDConstants.LENGTH);
  }

  @Override
  public void updateInputs(LEDsIOInputs inputs) {
    AddressableLEDJNI.setData(handle, inputs.buffer);
    // leds.setData(null);
  }
}
