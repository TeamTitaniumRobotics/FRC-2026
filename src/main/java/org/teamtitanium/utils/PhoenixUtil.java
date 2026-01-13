package org.teamtitanium.utils;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

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

  /***
   * TalonFX motor controller simulation.
   */
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = instances++;

      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  /***
   * TalonFX motor controller simulation with a remote CANcoder simulation.
   */
  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /***
   * Generates an array of odometry timestamps for the current simulation period.
   *
   * @return An array of timestamps corresponding to each odometry sub-tick within
   *         the current simulation period.
   */
  public static double[] getSimulationOdometryTimestamps() {
    final double[] odometryTimestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimestamps.length; i++) {
      odometryTimestamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimestamps;
  }

  /**
   *
   *
   * <h2>Regulates the {@link SwerveModuleConstants} for a single module.</h2>
   *
   * <p>This method applies specific adjustments to the {@link SwerveModuleConstants} for simulation
   * purposes. These changes have no effect on real robot operations and address known simulation
   * bugs:
   *
   * <ul>
   *   <li><strong>Inverted Drive Motors:</strong> Prevents drive PID issues caused by inverted
   *       configurations.
   *   <li><strong>Non-zero CanCoder Offsets:</strong> Fixes potential module state optimization
   *       issues.
   *   <li><strong>Steer Motor PID:</strong> Adjusts PID values tuned for real robots to improve
   *       simulation performance.
   * </ul>
   *
   * <h4>Note:This function is skipped when running on a real robot, ensuring no impact on constants
   * used on real robot hardware.</h4>
   */
  public static SwerveModuleConstants regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) return moduleConstants;

    // Apply simulation-specific adjustments to module constants
    return moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CanCoder inversion
        .withEncoderInverted(false)
        // Adjust steer motor PID gains for simulation
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSteerMotorGearRatio(16.0)
        // Adjust friction voltages
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.05))
        // Adjust steer inertia
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}
