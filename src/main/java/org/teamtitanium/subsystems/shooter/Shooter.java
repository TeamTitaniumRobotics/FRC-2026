package org.teamtitanium.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.teamtitanium.subsystems.shooter.backroller.BackRoller;
import org.teamtitanium.subsystems.shooter.backroller.BackRollerConstants;
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
        () -> RPM.of(ShotCalculator.getInstance().getParameters().flywheelIdleRPM()),
        () -> HoodConstants.STOW_ANGLE,
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().turretAngleRots())),
    AIM(
        () -> RPM.of(ShotCalculator.getInstance().getParameters().flywheelRPM()),
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().hoodAngleRots()),
        () -> Rotations.of(ShotCalculator.getInstance().getParameters().turretAngleRots())),
    EJECT(
        () -> FlywheelConstants.EJECT_VELOCITY,
        () -> HoodConstants.EJECT_ANGLE,
        () -> Degrees.of(-90.0));

    @Getter private final Supplier<AngularVelocity> flywheelVelocity;
    @Getter private final Supplier<Angle> hoodAngle;
    @Getter private final Supplier<Angle> turretAngle;

    public Supplier<AngularVelocity> getBackRollerVelocity() {
      return () -> {
        var shotParameters = ShotCalculator.getInstance().getParameters();

        double calculatedBackRollerRpm =
            getFlywheelVelocity().get().div(BackRollerConstants.WHEEL_RATIO).in(RPM);
        double targetBackRollerRpm =
            Double.isFinite(shotParameters.backRollerRPM())
                ? shotParameters.backRollerRPM()
                : calculatedBackRollerRpm;

        return RPM.of(
            MathUtil.clamp(targetBackRollerRpm, 0.0, BackRollerConstants.MAX_VELOCITY.in(RPM)));
      };
    }
  }

  private final Flywheel flywheel;
  private final BackRoller backRoller;
  private final Hood hood;
  private final Turret turret;

  @Getter
  @Setter
  @AutoLogOutput(key = "Shooter/State")
  private ShooterState state = ShooterState.STOW;

  @Getter
  @Setter
  @AutoLogOutput(key = "Shooter/TurretDisabled")
  private boolean turretDisabled = false;

  /**
   * Creates a new Shooter subsystem with the given flywheel, hood, and turret.
   *
   * @param flywheel
   * @param hood
   * @param turret
   */
  public Shooter(Flywheel flywheel, BackRoller backRoller, Hood hood, Turret turret) {
    this.flywheel = flywheel;
    this.backRoller = backRoller;
    this.hood = hood;
    this.turret = turret;

    flywheel.setDefaultCommand(
        Commands.either(
            flywheel.setVelocity(() -> ShooterState.AIM.getFlywheelVelocity().get()),
            flywheel.setVelocity(() -> state.getFlywheelVelocity().get()),
            RobotModeTriggers.autonomous()));
    // flywheel.setDefaultCommand(
    //     flywheel.setVelocity(() -> RPM.of(flywheel.flywheelConfigNumber1.get())));
    backRoller.setDefaultCommand(
        Commands.either(
            backRoller.setVelocity(() -> ShooterState.AIM.getBackRollerVelocity().get()),
            backRoller.setVelocity(() -> state.getBackRollerVelocity().get()),
            RobotModeTriggers.autonomous()));
    // backRoller.setDefaultCommand(
    //     Commands.either(
    //         backRoller.setVelocity(() -> RPM.of(BackRoller.configurableNumber.get())),
    //         backRoller.setVelocity(() -> state.getBackRollerVelocity().get()),
    //         () -> BackRoller.useConfigurableNumber.get() > 0.5));
    // backRoller.setDefaultCommand(
    //     backRoller.setVelocity(
    //         () ->
    //             RPM.of(
    //                 MathUtil.clamp(
    //                     flywheel.flywheelConfigNumber1.get() / BackRollerConstants.WHEEL_RATIO,
    //                     0.0,
    //                     BackRollerConstants.MAX_VELOCITY.in(RPM)))));
    // hood.setDefaultCommand(hood.setPosition(() -> Degrees.of(hood.hoodConfigNumber1.get())));
    hood.setDefaultCommand(hood.setPosition(() -> state.getHoodAngle().get()));
    // turret.setDefaultCommand(
    //     turret.setPosition(() -> Degrees.of(turret.turretConfigNumber1.get())));
    turret.setDefaultCommand(
        Commands.either(
            turret.setPosition(() -> state.getTurretAngle().get()),
            turret.setPosition(() -> turret.getPosition()),
            () -> !turretDisabled));
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public Trigger atSetpoint() {
    return hood.atSetpoint()
        .and(flywheel.atSetpoint())
        .and(turret.atSetpoint())
        .and(backRoller.atSetpoint());
  }

  @AutoLogOutput(key = "Shooter/TurretWrapping")
  public Trigger turretIsWrapping() {
    return turret.isWrapping();
  }

  public Angle getTurretAngle() {
    return turret.getPosition();
  }
}
