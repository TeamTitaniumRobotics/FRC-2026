package org.teamtitanium.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.HubTracker;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.virtualsubsystem.VirtualSubsystem;

public class LEDs extends VirtualSubsystem {
  private static LEDs instance = null;

  public static LEDs getInstance() {
    return instance;
  }

  @Setter private boolean lowBatteryAlert = false;
  @Setter private boolean coastOverride = false;
  @Setter private BooleanSupplier inShotTolerance;
  @Setter private boolean autoWinnerNotSet = false;
  @Setter private boolean eStopped = false;
  private Optional<Alliance> alliance = Optional.empty();

  public double shiftNearEndTime = 5.0;

  private final LEDsIO io;
  private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.LENGTH);
  private Field bufferField = null;

  public LEDs(LEDsIO io) {
    this.io = io;
    instance = this;

    try {
      bufferField = AddressableLEDBuffer.class.getDeclaredField("m_buffer");
      bufferField.setAccessible(true);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);

    // if (DriverStation.isFMSAttached()) {
    alliance = DriverStation.getAlliance();
    // }

    if (DriverStation.isEStopped()) {
      eStopped = true;
    }

    solid(Color.kBlack);

    if (eStopped) {
      solid(Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (coastOverride) {
        solid(Color.kWhite);
      } else if (lowBatteryAlert) {
        strobe(Color.kOrangeRed, Color.kBlack, LEDConstants.STROBE_DURATION);
      } else {
        if (alliance.isEmpty()) {
          // wave(
          //     Color.kGold,
          //     Color.kDarkBlue,
          //     LEDConstants.WAVE_DISABLED_CYCLE_LENGTH,
          //     LEDConstants.WAVE_DISABLED_DURATION);
          stripes(List.of(Color.kRed, Color.kWhite, Color.kDarkBlue), 3, 2.0);
        } else {
          Alliance currentAlliance = alliance.get();
          wave(
              currentAlliance == Alliance.Blue ? Color.kBlue : Color.kRed,
              Color.kBlack,
              LEDConstants.WAVE_DISABLED_CYCLE_LENGTH,
              LEDConstants.WAVE_DISABLED_DURATION);
        }
      }
    } else if (DriverStation.isAutonomous()) {
      wave(
          Color.kGold,
          Color.kDarkBlue,
          LEDConstants.WAVE_FAST_CYCLE_LENGTH,
          LEDConstants.WAVE_FAST_DURATION);
    } else {
      if (autoWinnerNotSet) {
        strobe(Color.kWhite, Color.kRed, LEDConstants.STROBE_DURATION);
      } else if (HubTracker.getOffsetShiftInfo().remainingTime() <= shiftNearEndTime) {
        wave(
            Color.kWhite,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Color.kBlue
                : Color.kRed,
            LEDConstants.WAVE_FAST_CYCLE_LENGTH,
            LEDConstants.WAVE_FAST_DURATION);
      } else if (inShotTolerance.getAsBoolean()) {
        wave(
            Color.kGreen,
            Color.kWhite,
            LEDConstants.WAVE_FAST_CYCLE_LENGTH,
            LEDConstants.WAVE_FAST_DURATION);
      }
    }

    if (bufferField == null) {
      return;
    }
    try {
      inputs.buffer = (byte[]) bufferField.get(buffer);
    } catch (Exception e) {
      e.printStackTrace();
    }

    LoggedTracer.record("LEDs");
  }

  private void solid(Color color) {
    if (color != null) {
      for (int i = 0; i < LEDConstants.LENGTH; i++) {
        setLED(i, color);
      }
    }
  }

  private void strobe(Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    if ((c1On && c1 == null) || (!c1On && c2 == null)) {
      return;
    }
    solid(c1On ? c1 : c2);
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = LEDConstants.LENGTH - 1; i >= 0.0; i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), LEDConstants.WAVE_EXPONENT) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.WAVE_EXPONENT) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = LEDConstants.LENGTH - 1; i >= 0; i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      setLED(i, colors.get(colorIndex));
    }
  }

  private void setLED(int index, Color color) {
    try {
      buffer.setRGB(
          index,
          (int) (color.green * 255 * LEDConstants.MAX_BRIGHTNESS),
          (int) (color.red * 255 * LEDConstants.MAX_BRIGHTNESS),
          (int) (color.blue * 255 * LEDConstants.MAX_BRIGHTNESS));
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
