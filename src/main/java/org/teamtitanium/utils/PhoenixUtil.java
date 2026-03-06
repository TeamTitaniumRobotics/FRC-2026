package org.teamtitanium.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Utility class for Phoenix devices */
public final class PhoenixUtil {
  /***
   * Tries to execute a command until it returns StatusCode.OK or reaches max
   * attempts.
   * Generally used to apply configurations to Phoenix devices that may fail due
   * to CAN bus issues.
   *
   * @param maxAttempts The maximum number of attempts to try
   * @param command     The command to execute
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error == StatusCode.OK) {
        return;
      }
    }
  }

  /** Status signals for synchronized refresh to decrease loop times */
  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /***
   * Registers status signals for synchronized refresh to decrease loop times
   *
   * @param bus     The CAN bus type of the device
   * @param signals The status signals to register
   */
  public static void registerSignals(CANBus bus, BaseStatusSignal... signals) {
    if (bus.isNetworkFD()) {
      var newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      var newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  /***
   * Refreshes all registered status signals
   */
  public static void refreshAll() {
    if (canivoreSignals.length > 0) {
      BaseStatusSignal.refreshAll(canivoreSignals);
    }
    if (rioSignals.length > 0) {
      BaseStatusSignal.refreshAll(rioSignals);
    }
  }
}
