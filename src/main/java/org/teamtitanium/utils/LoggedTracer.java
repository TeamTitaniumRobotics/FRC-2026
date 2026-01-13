package org.teamtitanium.utils;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class LoggedTracer {
  private static double startTime = -1.0;

  public static void reset() {
    startTime = Timer.getFPGATimestamp();
  }

  public static void record(String epochName) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
    startTime = now;
  }
}
