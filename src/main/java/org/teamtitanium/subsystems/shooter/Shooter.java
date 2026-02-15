package org.teamtitanium.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel;
import org.teamtitanium.subsystems.shooter.hood.Hood;
import org.teamtitanium.subsystems.shooter.turret.Turret;

public class Shooter {
  private final Turret turret;
  private final Hood hood;
  private final Flywheel flywheel;

  public Shooter(Turret turret, Hood hood, Flywheel flywheel) {
    this.turret = turret;
    this.hood = hood;
    this.flywheel = flywheel;
  }

  public Command stow() {
    return Commands.parallel(turret.stow(), hood.stow(), flywheel.idle());
  }

  public Command aim() {
    return Commands.parallel(turret.track(), hood.aim(), flywheel.shoot());
  }

  public Trigger atSetpoint() {
    return turret.atSetpoint().and(hood.atSetpoint()).and(flywheel.atSetpoint());
  }
}
