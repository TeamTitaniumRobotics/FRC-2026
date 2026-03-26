package org.teamtitanium.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class HubTracker {
  public enum Shift {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED
  }

  private static Timer shiftTimer = new Timer();
  private static final Shift[] shifts = Shift.values();

  private static final double[] shiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
  private static final double[] shiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};

  private static final double minFuelCountDelay = 1.0;
  private static final double maxFuelCountDelay = 2.0;
  private static final double shiftEndCountDelay = 3.0;
  private static final double minToF = 0.95;
  private static final double maxToF = 0.95;
  private static final double beforeActiveFudge = -1 * (minToF + minFuelCountDelay);
  private static final double endingActiveFudge =
      shiftEndCountDelay + -1 * (maxToF + maxFuelCountDelay);

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};
  private static final double timeResetThreshold = 3.0;
  private static double shiftTimerOffset = 0.0;
  @Setter private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static Optional<Boolean> getAllianceWinOverride() {
    return allianceWinOverride.get();
  }

  public static Alliance getFirstActiveAlliance() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    Optional<Boolean> winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        return Alliance.Blue;
      } else if (character == 'B') {
        return Alliance.Red;
      }
    }

    Logger.recordOutput("HubTracker/GameMessage", message);

    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  public static void initialize() {
    shiftTimerOffset = 0.0;
    shiftTimer.restart();
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double timerValue = shiftTimer.get();
    double currentTime = timerValue - shiftTimerOffset;
    double stateTimeElapsed = currentTime;
    double stateTimeRemaining = 0.0;
    boolean active = false;
    Shift currentShift = Shift.DISABLED;
    double fieldTeleopTime = 140.0 - DriverStation.getMatchTime();

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = Shift.AUTO;
    } else if (DriverStation.isEnabled()) {
      if (Math.abs(fieldTeleopTime - currentTime) >= timeResetThreshold
          && fieldTeleopTime <= 135
          && DriverStation.isFMSAttached()) {
        shiftTimerOffset += currentTime - fieldTeleopTime;
        currentTime = timerValue + shiftTimerOffset;
      }
      int currentShiftIndex = -1;
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shifts[currentShiftIndex];
    }

    ShiftInfo shiftInfo = new ShiftInfo(currentShift, stateTimeElapsed, stateTimeRemaining, active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getOffsetShiftInfo() {
    boolean[] shiftSchedule = getSchedule();

    if (shiftSchedule[1]) {
      double[] offsetShiftStartTimes = {
        0.0,
        10.0,
        35.0 + endingActiveFudge,
        60.0 + beforeActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + beforeActiveFudge
      };
      double[] offsetShiftEndTimes = {
        10.0,
        35.0 + endingActiveFudge,
        60.0 + beforeActiveFudge,
        85.0 + endingActiveFudge,
        110.0 + beforeActiveFudge,
        140.0
      };
      return getShiftInfo(shiftSchedule, offsetShiftStartTimes, offsetShiftEndTimes);
    }
    double[] offsetShiftStartTimes = {
      0.0,
      10.0 + endingActiveFudge,
      35.0 + beforeActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + beforeActiveFudge,
      110.0
    };
    double[] offsetShiftEndTimes = {
      10.0 + endingActiveFudge,
      35.0 + beforeActiveFudge,
      60.0 + endingActiveFudge,
      85.0 + beforeActiveFudge,
      110.0,
      140.0
    };
    return getShiftInfo(shiftSchedule, offsetShiftStartTimes, offsetShiftEndTimes);
  }

  public record ShiftInfo(
      Shift currentShift, double elapsedTime, double remainingTime, boolean active) {}
}
