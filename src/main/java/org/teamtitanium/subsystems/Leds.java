package org.teamtitanium.subsystems;

public class Leds {
  private static Leds instance = null;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }
}
