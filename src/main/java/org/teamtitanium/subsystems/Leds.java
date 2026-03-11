package org.teamtitanium.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import org.teamtitanium.subsystems.Superstructure.SuperstructureState;
import org.teamtitanium.utils.virtualsubsystem.VirtualSubsystem;

public class Leds extends VirtualSubsystem {
  private static Leds instance = null;
  private final Dimensionless BRIGHTNESS = Percent.of(50);
  private final Frequency SCROLLSPEED = Percent.per(Seconds).of(100);
  private final Map<SuperstructureState, LEDStates> mappedStates = new HashMap<>();
  private final SuperstructureState currentState;
  private final AddressableLED led =
      new AddressableLED(0); // TODO: Make sure this is the right PWM port
  private final int LENGTH = 10; // TODO: set to number of leds
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LENGTH);

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  private Leds() {
    currentState = Superstructure.getState();
    if (SuperstructureState.values().length == LEDStates.values().length) {
      for (int i = 0; i < SuperstructureState.values().length; i++) {
        mappedStates.put(SuperstructureState.values()[i], LEDStates.values()[i]);
      }
    } else {
      System.out.println(
          "The number of LEDStates is not the same as the number of SuperstructureStates.");
    }
    led.setLength(LENGTH);
    led.start();
  }

  private enum LEDStates {
    IDLE(null),
    INTAKE(
        LEDPattern.solid(Color.kOrange)
            .scrollAtRelativeSpeed(instance.SCROLLSPEED)
            .atBrightness(instance.BRIGHTNESS)),
    SPIN_UP_SCORE(LEDPattern.solid(Color.kBlue).atBrightness(instance.BRIGHTNESS)),
    SCORE(
        LEDPattern.steps(Map.of(0.00, Color.kGreen, 0.50, Color.kYellow))
            .scrollAtRelativeSpeed(instance.SCROLLSPEED)
            .atBrightness(instance.BRIGHTNESS)),
    SPIN_UP_PASS(LEDPattern.solid(Color.kBlue).atBrightness(instance.BRIGHTNESS)),
    PASS(
        LEDPattern.steps(Map.of(0.00, Color.kBlack, 0.50, Color.kYellow))
            .scrollAtRelativeSpeed(instance.SCROLLSPEED)
            .atBrightness(instance.BRIGHTNESS)),
    EJECT(LEDPattern.solid(Color.kRed).blink(Seconds.of(2)).atBrightness(instance.BRIGHTNESS)),
    PREP_CLIMB(LEDPattern.solid(Color.kBlue).atBrightness(instance.BRIGHTNESS)),
    CLIMB(LEDPattern.solid(Color.kGold).atBrightness(instance.BRIGHTNESS)),
    CLIMB_L1(LEDPattern.solid(Color.kGold).atBrightness(instance.BRIGHTNESS)),
    DE_CLIMB_L1(LEDPattern.solid(Color.kBrown).atBrightness(instance.BRIGHTNESS));

    @Getter private LEDPattern animation;

    private LEDStates(LEDPattern animation) {
      this.animation =
          (animation == null)
              ? LEDPattern.solid(Color.kRed)
                  .breathe(Seconds.of(3))
                  .atBrightness(instance.BRIGHTNESS)
              : animation;
    }
  }

  @Override
  public void periodic() {
    if (currentState.toString() != Superstructure.getState().toString()) {
      try {
        mappedStates.get(Superstructure.getState()).getAnimation().applyTo(buffer);
        led.setData(buffer);
      } catch (Exception e) {
        System.out.println(e);
      }
    }
  }

  // public void simulationPeriodic() {

  // }

}
