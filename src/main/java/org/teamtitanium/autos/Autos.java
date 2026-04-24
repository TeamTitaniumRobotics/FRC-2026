package org.teamtitanium.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.autos.Paths.Path;
import org.teamtitanium.subsystems.swerve.Swerve;

public class Autos {
  private final Swerve swerve;

  @AutoLogOutput(key = "Autos/CurrentState")
  private boolean autoIntake = false;

  private boolean autoScore = false;

  public Autos(Swerve swerve) {
    this.swerve = swerve;
  }

  public Command rightDoublePassBump() {

    return followPath(Paths.RTS_RFM.getPath());
  }

  private Command followPath(Paths.Path path) {
    return switch (path.action()) {
      case INTAKE -> intakePath(path);
      case SCORE -> scorePath(path);
      case NOTHING -> swerve.pathBuilder.build(path.getPath());
      default -> swerve.pathBuilder.build(path.getPath());
    };
  }

  private Command setAutoIntake(boolean value) {
    return Commands.runOnce(
        () -> {
          autoIntake = value;
          Logger.recordOutput("Autos/CurrentState", "Auto Intake: " + value);
        });
  }

  private Command setAutoScore(boolean value) {
    return Commands.runOnce(
        () -> {
          autoScore = value;
          Logger.recordOutput("Autos/CurrentState", "Auto Score: " + value);
        });
  }

  private Command intakePath(Path path) {
    return Commands.deadline(
            swerve.pathBuilder.build(path.getPath()), setAutoIntake(true), setAutoScore(false))
        .andThen(setAutoIntake(false));
  }

  private Command scorePath(Path path) {
    return Commands.deadline(
            swerve.pathBuilder.build(path.getPath()), setAutoIntake(false), setAutoScore(true))
        .andThen(setAutoScore(false));
  }
}
