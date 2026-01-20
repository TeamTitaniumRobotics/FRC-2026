// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.teamtitanium.utils;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

@SuppressWarnings("unused")
public class LoggedTunableBoolean implements Supplier<Boolean> {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private boolean defaultValue = false;
  private LoggedNetworkBoolean dashboardBoolean;
  private Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /***
   * Creates a LoggedTunableNumber that can be modified from the dashboard.
   *
   * @param dashboardKey The key to use on the dashboard.
   */
  public LoggedTunableBoolean(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /***
   * Creates a LoggedTunableNumber that can be modified from the dashboard, with
   * a default value.
   *
   * @param dashboardKey The key to use on the dashboard.
   * @param defaultValue The default value to set.
   */
  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /***
   * Sets the default value for this tunable number. The default value is only set
   * once, and subsequent calls to this method will have no effect.
   *
   * @param defaultValue The default value to set.
   */
  public void initDefault(boolean defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.tuningMode && !Constants.disableHAL) {
        dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  @Override
  public Boolean get() {
    if (!hasDefault) {
      return false;
    } else {
      return Constants.tuningMode && !Constants.disableHAL ? dashboardBoolean.get() : defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    boolean currentValue = get();
    Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<Boolean[]> action, LoggedTunableBoolean... tuneableBoolean) {
    if (Arrays.stream(tuneableBoolean).anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id))) {
      action.accept(
          Arrays.stream(tuneableBoolean).map(LoggedTunableBoolean::get).toArray(Boolean[]::new));
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, LoggedTunableBoolean... tuneableNumbers) {
    ifChanged(id, values -> action.run(), tuneableNumbers);
  }
}
