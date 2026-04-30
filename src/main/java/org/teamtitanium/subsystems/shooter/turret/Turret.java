package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.Robot;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import yams.units.EasyCRTConfig.CrtGearPair;

public class Turret extends SubsystemBase {
  private static final int dashboardLogDivisor = 5;
  private static final double wrapBoundaryEntryMarginRad = Math.toRadians(12.0);
  private static final double wrapBoundaryExitMarginRad = Math.toRadians(18.0);
  private static final double wrapMinCommandDeltaRad = Math.toRadians(25.0);

  // Real-time tunable PID gains
  private final LoggedTunableNumber turretkP =
      new LoggedTunableNumber("Shooter/Turret/kP", TURRET_GAINS.kP());
  private final LoggedTunableNumber turretkI =
      new LoggedTunableNumber("Shooter/Turret/kI", TURRET_GAINS.kI());
  private final LoggedTunableNumber turretkD =
      new LoggedTunableNumber("Shooter/Turret/kD", TURRET_GAINS.kD());
  private final LoggedTunableNumber turretkS =
      new LoggedTunableNumber("Shooter/Turret/kS", TURRET_GAINS.kS());
  private final LoggedTunableNumber turretkV =
      new LoggedTunableNumber("Shooter/Turret/kV", TURRET_GAINS.kV());
  private final LoggedTunableNumber turretkA =
      new LoggedTunableNumber("Shooter/Turret/kA", TURRET_GAINS.kA());

  // Real-time tunable constraints
  private final LoggedTunableNumber turretMaxVelocity =
      new LoggedTunableNumber("Shooter/Turret/MaxVelocity", TURRET_CONSTRAINTS.maxVelocity());
  private final LoggedTunableNumber turretMaxAcceleration =
      new LoggedTunableNumber(
          "Shooter/Turret/MaxAcceleration", TURRET_CONSTRAINTS.maxAcceleration());

  public final LoggedTunableNumber turretConfigNumber1 =
      new LoggedTunableNumber("Shooter/Turret/ConfigNumber1", 0.0);
  public final LoggedTunableNumber turretConfigNumber2 =
      new LoggedTunableNumber("Shooter/Turret/ConfigNumber2", 0.0);

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private final EasyCRT easyCRT;
  private final EasyCRTConfig crtConfig;

  @AutoLogOutput(key = "Shooter/Turret/IsZeroed")
  private boolean isZeroed = false;

  private final Alert crtErrorAlert = new Alert("Turret CRT Error", Alert.AlertType.kWarning);

  @AutoLogOutput(key = "Shooter/Turret/TargetPositionRots")
  private double targetPositionRots = 0.0;

  @AutoLogOutput(key = "Shooter/Turret/Wrap/IsWrapping")
  private boolean isWrapping = false;

  @AutoLogOutput(key = "Shooter/Turret/Wrap/DetectedThisCycle")
  private boolean wrapDetectedThisCycle = false;

  private int dashboardLogCounter = 0;

  private Trigger atSetpoint =
      new Trigger(() -> Math.abs(inputs.positionRots - targetPositionRots) < ANGLE_TOLERANCE_ROTS);

  @AutoLogOutput(key = "Shooter/Turret/Wrapping")
  private Trigger wrappingTrigger =
      new Trigger(
          () -> Math.abs(inputs.positionRots - targetPositionRots) < WRAPPING_TOLERANCE_ROTS);

  static record WrapEvaluation(boolean wrappingThisCycle, boolean wrappingAfterUpdate) {}

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
            .withMatchTolerance(Rotations.of(0.01))
            // .withAbsoluteEncoderInversions(true, true)
            // .withAbsoluteEncoderOffsets(Rotations.of(0.035400), Rotations.of(-0.275635))
            .withCrtGearRecommendationConstraints(1.2, 15, 45, 30);
    this.easyCRT = new EasyCRT(crtConfig);

    Robot.getCoastOverrideTrigger()
        .onTrue(runOnce(() -> this.setBrakeMode(false)).ignoringDisable(true))
        .onFalse(runOnce(() -> this.setBrakeMode(true)).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    // Process turret inputs and ouputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);

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
    if (!isZeroed && Robot.isInitializing()) {
      zeroTurretCRT();
    }

    boolean shouldLogDashboardData = dashboardLogCounter++ % dashboardLogDivisor == 0;
    if (Constants.tuningMode) {
      shouldLogDashboardData = true;
    }
    if (shouldLogDashboardData) {
      SmartDashboard.putNumber("Dashboard/Turret/Angle", getPosition().in(Degrees));
    }

    // Log the turret loop time
    LoggedTracer.record("Shooter/Turret");
  }

  public Angle getTargetAngle(Angle targetAngle, Angle currentAngle) {
    double minRad = MIN_ANGLE.in(Radians);
    double maxRad = MAX_ANGLE.in(Radians);

    double targetRad = targetAngle.in(Radians);
    double currentRad = currentAngle.in(Radians);

    double normalizedTarget =
        targetRad - (2 * Math.PI) * Math.floor((targetRad - minRad) / (2 * Math.PI));

    double bestTarget = Double.NaN;
    double bestDistance = Double.MAX_VALUE;

    for (int offset = -2; offset <= 2; offset++) {
      double candidate = normalizedTarget + offset * 2 * Math.PI;
      if (candidate >= minRad - 1e-9 && candidate <= maxRad + 1e-9) {
        candidate = Math.max(minRad, Math.min(maxRad, candidate));
        double distance = Math.abs(candidate - currentRad);
        if (distance < bestDistance) {
          bestDistance = distance;
          bestTarget = candidate;
        }
      }
    }

    if (Double.isNaN(bestTarget)) {
      double distToMin = Math.abs(currentRad - minRad);
      double distToMax = Math.abs(currentRad - maxRad);
      bestTarget = distToMin < distToMax ? minRad : maxRad;
    }

    updateWrapState(currentRad, bestTarget);

    // Logger.recordOutput("Shooter/Turret/DeltaAngle", bestTarget - currentRad);
    // Logger.recordOutput("Shooter/Turret/OptimalAngle", bestTarget);

    return Radians.of(bestTarget);
  }

  private void updateWrapState(double currentRad, double commandedTargetRad) {
    WrapEvaluation evaluation =
        evaluateWrappingState(
            currentRad,
            commandedTargetRad,
            isWrapping,
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            wrapBoundaryEntryMarginRad,
            wrapBoundaryExitMarginRad,
            wrapMinCommandDeltaRad,
            ANGLE_TOLERANCE_ROTS * 2.0 * Math.PI);

    isWrapping = evaluation.wrappingAfterUpdate();
    wrapDetectedThisCycle = evaluation.wrappingThisCycle();

    double commandedDeltaRad = Math.abs(commandedTargetRad - currentRad);

    Logger.recordOutput("Shooter/Turret/Wrap/CurrentAngleDeg", Math.toDegrees(currentRad));
    Logger.recordOutput(
        "Shooter/Turret/Wrap/CommandedTargetDeg", Math.toDegrees(commandedTargetRad));
    Logger.recordOutput("Shooter/Turret/Wrap/CommandedDeltaDeg", Math.toDegrees(commandedDeltaRad));
  }

  static WrapEvaluation evaluateWrappingState(
      double currentRad,
      double commandedTargetRad,
      boolean wasWrapping,
      double minRad,
      double maxRad,
      double boundaryEntryMarginRad,
      double boundaryExitMarginRad,
      double minCommandDeltaRad,
      double nearTargetDeltaRad) {
    double commandedDeltaRad = Math.abs(commandedTargetRad - currentRad);

    boolean nearCurrentMin = currentRad - minRad <= boundaryEntryMarginRad;
    boolean nearCurrentMax = maxRad - currentRad <= boundaryEntryMarginRad;
    boolean nearTargetMin = commandedTargetRad - minRad <= boundaryEntryMarginRad;
    boolean nearTargetMax = maxRad - commandedTargetRad <= boundaryEntryMarginRad;

    boolean wrappingThisCycle =
        commandedDeltaRad > minCommandDeltaRad
            && ((nearCurrentMax && nearTargetMin) || (nearCurrentMin && nearTargetMax));

    boolean wrappingAfterUpdate = wasWrapping;
    if (wrappingThisCycle) {
      wrappingAfterUpdate = true;
    } else if (wrappingAfterUpdate) {
      boolean nearBoundaryForExit =
          (currentRad - minRad <= boundaryExitMarginRad)
              || (maxRad - currentRad <= boundaryExitMarginRad)
              || (commandedTargetRad - minRad <= boundaryExitMarginRad)
              || (maxRad - commandedTargetRad <= boundaryExitMarginRad);
      boolean nearCommandedTarget = commandedDeltaRad < nearTargetDeltaRad;

      if (!nearBoundaryForExit || nearCommandedTarget) {
        wrappingAfterUpdate = false;
      }
    }

    return new WrapEvaluation(wrappingThisCycle, wrappingAfterUpdate);
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

  public Command stop() {
    return run(() -> io.stop());
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
  @AutoLogOutput(key = "Shooter/Turret/AtSetpoint")
  public Trigger atSetpoint() {
    return atSetpoint;
  }

  public Trigger isWrapping() {
    return new Trigger(() -> false);
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
      double zeroRots = zeroAngle.get().plus(CRT_OFFSET).in(Rotations);
      io.setMotorPosition(zeroRots);
      isZeroed = true;
      if (crtErrorAlert.get()) {
        crtErrorAlert.set(false);
      }
      Logger.recordOutput("Turret/CRT/ZeroAngleRots", zeroRots);
    } else {
      crtErrorAlert.set(true);
      Logger.recordOutput(
          "Turret/CRT/LastStatus",
          "CRT failed to find a valid zero angle, check CANcoder readings and CRT configuration!");
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
