package org.teamtitanium.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamtitanium.RobotState;
import org.teamtitanium.subsystems.shooter.flywheel.Flywheel;
import org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants;
import org.teamtitanium.subsystems.shooter.hood.Hood;
import org.teamtitanium.subsystems.shooter.hood.HoodConstants;
import org.teamtitanium.subsystems.shooter.turret.Turret;

/** Shooter subsystem for controlling the turret, hood, and flywheel. */
public class Shooter {
  /** States for the shooter. */
  @RequiredArgsConstructor
  public enum ShooterState {
    STOW(
        () -> RPM.of(ShotCalculator.getInstance().getParameters().flywheelRPM()),
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().hoodAngleRots()),
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().turretAngleRots())),
    AIM(
        () -> RPM.of(ShotCalculator.getInstance().getParameters().flywheelRPM()),
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().hoodAngleRots()),
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().turretAngleRots())),
    EJECT(
        () -> FlywheelConstants.EJECT_VELOCITY,
        () -> HoodConstants.EJECT_ANGLE,
        () -> Degrees.of(0.0));

    @Getter private final Supplier<AngularVelocity> flywheelVelocity;
    @Getter private final Supplier<Angle> hoodAngle;
    @Getter private final Supplier<Angle> turretAngle;
  }

  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;

  @Getter
  @Setter
  @AutoLogOutput(key = "Shooter/State")
  private ShooterState state = ShooterState.STOW;

  /**
   * Override trigger: when active, the hood will stow regardless of the current shooter state. Used
   * for auto-stow when going under the trench, etc.
   */
  @AutoLogOutput(key = "Shooter/HoodStowOverride")
  @Setter
  private Trigger hoodStowOverride = RobotState.getInstance().underTrench;

  /**
   * Creates a new Shooter subsystem with the given flywheel, hood, and turret.
   *
   * @param flywheel
   * @param hood
   * @param turret
   */
  public Shooter(Flywheel flywheel, Hood hood, Turret turret) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;

    flywheel.setDefaultCommand(flywheel.setVelocity(() -> state.getFlywheelVelocity().get()));
    // flywheel.setDefaultCommand(
    //     flywheel.setVelocity(() -> RPM.of(flywheel.flywheelConfigNumber1.get())));
    // hood.setDefaultCommand(hood.setPosition(() -> Degrees.of(hood.hoodConfigNumber1.get())));
    hood.setDefaultCommand(
        Commands.either(
            hood.setPosition(() -> HoodConstants.STOW_ANGLE),
            hood.setPosition(() -> state.getHoodAngle().get()),
            () -> hoodStowOverride.getAsBoolean()));
    // turret.setDefaultCommand(
    //     turret.setPosition(() -> Degrees.of(turret.turretConfigNumber1.get())));
    turret.setDefaultCommand(turret.setPosition(() -> state.getTurretAngle().get()));
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public Trigger atSetpoint() {
    return hood.atSetpoint().and(flywheel.atSetpoint()).and(turret.atSetpoint());
  }

  public Angle getTurretAngle() {
    return turret.getPosition();
  }
}
