package org.teamtitanium.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel;
import org.teamtitanium.subsystems.shooter.hood.Hood;
import org.teamtitanium.subsystems.shooter.turret.Turret;

public class Shooter {
  /** Independent states for the shooter subsystem group. */
  public enum ShooterState {
    STOW,
    AIM,
    EJECT
  }

  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  @Getter
  @Setter
  @AutoLogOutput(key = "Shooter/State")
  private ShooterState state = ShooterState.STOW;

  /**
   * Override trigger: when active, the hood will stow regardless of the current shooter state. Used
   * for auto-stow when going under the trench, etc.
   */
  @Setter private Trigger hoodStowOverride = new Trigger(() -> false);

  public Shooter(Turret turret, Hood hood, Flywheel flywheel) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
  }

  /**
   * Returns a command that continuously resolves the current {@link ShooterState} (plus any active
   * overrides) into the appropriate turret / hood / flywheel commands. This should be run as a
   * long-lived command.
   */
  public Command applySubStates() {
    return Commands.run(
            () -> {
              Logger.recordOutput("Shooter/State", state.name());
            })
        .alongWith(
            Commands.select(
                    java.util.Map.of(
                        ShooterState.STOW, stow(),
                        ShooterState.AIM,
                            Commands.either(
                                // When hoodStowOverride is active, stow the hood but keep
                                // turret tracking and flywheel spinning
                                Commands.parallel(turret.track(), hood.stow(), flywheel.shoot()),
                                aim(),
                                hoodStowOverride),
                        ShooterState.EJECT, eject()),
                    () -> state)
                .repeatedly())
        .ignoringDisable(true)
        .withName("Shooter.ApplySubStates");
  }

  // ---- Low-level command factories (package-visible for direct use in autos) ----

  public Command stow() {
    return Commands.parallel(turret.stow(), hood.stow(), flywheel.idle());
  }

  public Command aim() {
    return Commands.parallel(turret.track(), hood.aim(), flywheel.shoot());
  }

  public Command eject() {
    return Commands.parallel(turret.stow(), hood.stow(), flywheel.eject());
  }

  public Trigger atSetpoint() {
    return turret.atSetpoint().and(hood.atSetpoint()).and(flywheel.atSetpoint());
  }
}
