package org.teamtitanium.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class HubTracker {
  private static String currentGameData = "";

  /**
   * Determines if the active hub matches the team's alliance.
   *
   * @return true if the active hub is the same as the team's alliance, false otherwise or if game
   *     data is unavailable.
   */
  public static boolean isAllianceHubActive() {
    Alliance activeAlliance = getActiveAlliance().orElse(null);
    Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (activeAlliance == null) {
      return false;
    }

    return activeAlliance == teamAlliance;
  }

  /**
   * Gets the currently active alliance based on match time and game data.
   *
   * @return The active Alliance (Red or Blue), or null if game data is unavailable.
   */
  public static Optional<Alliance> getActiveAlliance() {
    if (DriverStation.isAutonomous()) {
      return Optional.of(DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    var currentTime = DriverStation.getMatchTime();

    if (currentGameData.length() == 0) {
      currentGameData = DriverStation.getGameSpecificMessage();
      if (currentGameData.length() == 0) {
        return Optional.empty();
      }
    }

    Alliance initialAlliance =
        DriverStation.getGameSpecificMessage().charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

    if (currentTime >= 130 || currentTime < 30) {
      return Optional.of(DriverStation.getAlliance().orElse(Alliance.Blue));
    } else if (currentTime >= 105 || (currentTime < 80 && currentTime >= 55)) {
      return Optional.of(initialAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red);
    } else {
      return Optional.of(initialAlliance);
    }
  }
}
