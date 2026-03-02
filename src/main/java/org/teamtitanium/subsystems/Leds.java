package org.teamtitanium.subsystems;

import org.teamtitanium.utils.VirtualSubsystem;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;

import lombok.Getter;
import lombok.Setter;

public class Leds extends VirtualSubsystem{
  private static Leds instance = null;
  private final CANdle candle;
  private final Superstructure superstructure;
  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  private Leds(Superstructure superstructure) {
    candle = new CANdle(0); //TODO: replace with actual id
    this.superstructure = superstructure;
  }

  public enum LEDStates{
    IDLE(null),
    INTAKE(null),
    PREPPED(null),
    SPIN_UP_SCORE(null),
    SCORE(null),
    SCORE_THROUGH(null),
    SPIN_UP_PASS(null),
    PASS(null),
    PASS_THROUGH(null),
    PREP_HUB(null),
    SCORE_HUB(null),
    PREP_OUTPOST(null),
    SCORE_OUTPOST(null),
    EJECT(null),
    PREP_CLIMB(null),
    CLIMB(null),
    CLIMB_L1(null),
    DE_CLIMB_L1(null);

    private ControlRequest animation;

    private LEDStates(ControlRequest animation) {
      this.animation = animation;
    }
  }

  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      
  }

  // public void simulationPeriodic() {

  // }
}
