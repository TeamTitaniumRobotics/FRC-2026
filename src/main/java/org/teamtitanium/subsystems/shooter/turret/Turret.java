package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import yams.units.EasyCRTConfig.CrtGearPair;

public class Turret extends SubsystemBase {
  // Real-time tunable PID gains
  private final LoggedTunableNumber turretkP =
      new LoggedTunableNumber("Turret/kP", TURRET_GAINS.kP());
  private final LoggedTunableNumber turretkI =
      new LoggedTunableNumber("Turret/kI", TURRET_GAINS.kI());
  private final LoggedTunableNumber turretkD =
      new LoggedTunableNumber("Turret/kD", TURRET_GAINS.kD());
  private final LoggedTunableNumber turretkS =
      new LoggedTunableNumber("Turret/kS", TURRET_GAINS.kS());
  private final LoggedTunableNumber turretkV =
      new LoggedTunableNumber("Turret/kV", TURRET_GAINS.kV());
  private final LoggedTunableNumber turretkA =
      new LoggedTunableNumber("Turret/kA", TURRET_GAINS.kA());

  // Real-time tunable constraints
  private final LoggedTunableNumber turretMaxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity", TURRET_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber turretMaxAcceleration =
      new LoggedTunableNumber("Turret/MaxAcceleration", TURRET_CONSTRAINTS.maxAcceleration());

  public final LoggedTunableNumber turretConfigNumber1 =
      new LoggedTunableNumber("Turret/ConfigNumber1", 0.0);
  public final LoggedTunableNumber turretConfigNumber2 =
      new LoggedTunableNumber("Turret/ConfigNumber2", 0.0);

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final EasyCRT easyCRT;
  private final EasyCRTConfig crtConfig;

  @AutoLogOutput(key = "Turret/IsZeroed")
  private boolean isZeroed = false;

  private final Alert crtErrorAlert = new Alert("Turret CRT Error", Alert.AlertType.kWarning);

  @AutoLogOutput(key = "Turret/TargetPositionRots")
  private double targetPositionRots = 0.0;

  private Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.positionRots - targetPositionRots) < ANGLE_TOLERANCE_ROTS);

  /** Creates a new Turret subsystem. */
  public Turret(TurretIO io) {
    this.io = io;

    this.crtConfig =
        new EasyCRTConfig(
                () -> Rotations.of(inputs.cancoder1PositionRots),
                () -> Rotations.of(inputs.cancoder2PositionRots))
            .withCommonDriveGear(
                CANCODER_COMMON_RATIO,
                CANCODER_DRIVE_GEAR_TEETH,
                CANCODER_1_GEAR_TEETH,
                CANCODER_2_GEAR_TEETH)
            .withMechanismRange(MIN_ANGLE, MAX_ANGLE)
            .withMatchTolerance(Rotations.of(0.06))
            .withAbsoluteEncoderInversions(false, false)
            .withAbsoluteEncoderOffsets(Rotations.of(-0.4751), Rotations.of(0.2876))
            .withCrtGearRecommendationConstraints(1.2, 15, 45, 30);
    this.easyCRT = new EasyCRT(crtConfig);
  }

  @Override
  public void periodic() {
    // Process turret inputs and ouputs
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Check if PID or FF gains have changed
    if (turretkP.hasChanged(hashCode())
        || turretkI.hasChanged(hashCode())
        || turretkD.hasChanged(hashCode())
        || turretkS.hasChanged(hashCode())
        || turretkV.hasChanged(hashCode())
        || turretkA.hasChanged(hashCode())) {
      io.setGains(
          new Gains(
              turretkP.get(),
              turretkI.get(),
              turretkD.get(),
              turretkS.get(),
              turretkV.get(),
              0.0,
              turretkA.get()));
    }

    // Check if max velocity or acceleration has changed
    if (turretMaxVelocity.hasChanged(hashCode()) || turretMaxAcceleration.hasChanged(hashCode())) {
      io.setConstraints(new Constraints(turretMaxVelocity.get(), turretMaxAcceleration.get()));
    }

    // Attempt to zero the turret
    // if (!isZeroed && Robot.isInitializing()) {
    // zeroTurretCRT();
    // }

    // Log the turret loop time
    LoggedTracer.record("Turret");
  }

  public Angle getTargetAngle(Angle targetAngle, Angle currentAngle) {
    // Normalize target angle to [-180, 180] range
    double targetRad = targetAngle.in(Radians);
    while (targetRad > Math.PI) {
      targetRad -= 2 * Math.PI;
    }
    while (targetRad < -Math.PI) {
      targetRad += 2 * Math.PI;
    }

    double currentRad = currentAngle.in(Radians);

    // Calculate shortest angular distance
    double deltaAngleRad = targetRad - currentRad;
    if (deltaAngleRad > Math.PI) {
      deltaAngleRad -= 2 * Math.PI;
    } else if (deltaAngleRad < -Math.PI) {
      deltaAngleRad += 2 * Math.PI;
    }

    Logger.recordOutput("Turret/DeltaAngle", deltaAngleRad);

    // Calculate optimal target position
    double optimalAngleRad = currentRad + deltaAngleRad;
    Logger.recordOutput("Turret/OptimalAngle", optimalAngleRad);

    // Special case: if delta is exactly ±π (±180°), prefer the representation
    // that matches the original target to avoid oscillation
    if (Math.abs(Math.abs(deltaAngleRad) - Math.PI) < 0.001) {
      // Use the normalized target directly to be consistent
      optimalAngleRad = targetRad;
    }

    // Wrap to keep within [-π, π] range for continuous rotation tracking
    // But don't wrap if we're already very close to the boundary to avoid oscillation
    if (optimalAngleRad > Math.PI && Math.abs(optimalAngleRad - Math.PI) > 0.001) {
      optimalAngleRad -= 2 * Math.PI;
    } else if (optimalAngleRad < -Math.PI && Math.abs(optimalAngleRad + Math.PI) > 0.001) {
      optimalAngleRad += 2 * Math.PI;
    }

    return Radians.of(optimalAngleRad);
  }

  /**
   * Sets the turret to a position supplider
   *
   * @param positionRots target supplier position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(Supplier<Angle> position) {
    return run(() -> {
          targetPositionRots = getTargetAngle(position.get(), getPosition()).in(Rotations);
          io.setPosition(targetPositionRots);
        })
        .withName("Turret.SetPosition");
  }

  /**
   * Sets the turret to a position
   *
   * @param position target position for the turret
   * @return A command that repeatedly sets the turret to a position
   */
  public Command setPosition(Angle position) {
    return setPosition(() -> position);
  }

  /**
   * Runs the turret at a given voltage supplier
   *
   * @param voltage target voltage supplier
   * @return A command that repeatedly runs the turret at a voltage
   */
  public Command setVoltage(DoubleSupplier voltage) {
    return runEnd(() -> io.setVoltage(voltage.getAsDouble()), () -> io.setVoltage(0.0))
        .withName("Turret.SetVoltage");
  }

  /**
   * Runs the turret at a given voltage
   *
   * @param voltage target voltage
   * @return A command that repeatedly runs the turret at a voltage
   */
  public Command setVoltage(double voltage) {
    return setVoltage(() -> voltage);
  }

  /**
   * Gets the current position of the turret.
   *
   * @return The current turret angle
   */
  public Angle getPosition() {
    return Rotations.of(inputs.positionRots);
  }

  /**
   * Checks if the turret is at the target position.
   *
   * @return A trigger that is true if at target within tolerance
   */
  @AutoLogOutput(key = "Turret/AtSetpoint")
  public Trigger atSetpoint() {
    return atSetpoint;
  }

  /**
   * Sets the turret to brake mode or coast mode.
   *
   * @param enabled True for brake mode, false for coast mode
   */
  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public void zeroMotor() {
    io.setMotorPosition(0.0);
  }

  /** Zeros the turret using two CANcoder sensors and the Chinese Remainder Theorem. */
  public void zeroTurretCRT() {
    if (!inputs.motorConnected || !inputs.cancoder1Connected || !inputs.cancoder2Connected) {
      crtErrorAlert.set(true);
      Logger.recordOutput(
          "Turret/CRT/LastStatus", "Turret motor or CANcoders not connected, CRT aborted!");
      return;
    }

    Optional<Angle> zeroAngle = easyCRT.getAngleOptional();
    if (zeroAngle.isPresent()) {
      double zeroRots = zeroAngle.get().in(Rotations);
      // io.setMotorPosition(zeroRots);
      isZeroed = true;
      if (crtErrorAlert.get()) {
        crtErrorAlert.set(false);
      }
      Logger.recordOutput("Turret/CRT/ZeroAngleRots", zeroRots);
    } else {
      Logger.recordOutput("Turret/CRT/ZeroAngleRots", Double.NaN);
    }
    Logger.recordOutput("Turret/CRT/LastErrorRots", easyCRT.getLastErrorRotations());
    Logger.recordOutput("Turret/CRT/LastStatus", easyCRT.getLastStatus());
  }

  /** Checks the CRT values and logs them. Only for use in Sim. */
  public void checkCrtValues() {
    Optional<Angle> coverage = crtConfig.getUniqueCoverage();
    if (coverage.isPresent()) {
      Logger.recordOutput("Turret/CRT/CoverageDegrees", coverage.get().in(Degrees));
    } else {
      Logger.recordOutput("Turret/CRT/CoverageDegrees", Double.NaN);
    }

    Optional<CrtGearPair> gearPairOpt = crtConfig.getRecommendedCrtGearPair();
    if (gearPairOpt.isPresent()) {
      CrtGearPair gearPair = gearPairOpt.get();
      Logger.recordOutput("Turret/CRT/Gear1Teeth", gearPair.gearA());
      Logger.recordOutput("Turret/CRT/Gear2Teeth", gearPair.gearB());
      Logger.recordOutput("Turret/CRT/GCD", gearPair.gcd());
      Logger.recordOutput("Turret/CRT/LCM", gearPair.lcm());
      Logger.recordOutput("Turret/CRT/Iterations", gearPair.theoreticalIterations());
    } else {
      Logger.recordOutput("Turret/CRT/Gear1Teeth", Double.NaN);
      Logger.recordOutput("Turret/CRT/Gear2Teeth", Double.NaN);
      Logger.recordOutput("Turret/CRT/GCD", Double.NaN);
      Logger.recordOutput("Turret/CRT/LCM", Double.NaN);
      Logger.recordOutput("Turret/CRT/Iterations", Double.NaN);
    }
    Logger.recordOutput("Turret/CRT/CoverageGood", crtConfig.coverageSatisfiesRange());
  }
}
