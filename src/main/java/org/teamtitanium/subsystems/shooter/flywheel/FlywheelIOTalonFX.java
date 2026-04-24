package org.teamtitanium.subsystems.shooter.flywheel;

import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  protected final TalonFX leftMotor;
  protected final TalonFX rightMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withUpdateFreqHz(250.0).withEnableFOC(true);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> velocitySetpoint;
  private final List<StatusSignal<Voltage>> appliedVolts;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> temperature;

  private final Object controlLock = new Object();
  private final Object signalLock = new Object();
  private volatile double targetVelocityRps = 0.0;
  private volatile double measuredVelocityRps = 0.0;
  private volatile int activeSlot = 0;
  private volatile boolean velocityMode = false;

  private final Notifier gainNotifier;

  public FlywheelIOTalonFX() {
    leftMotor = new TalonFX(FLYWHEEL_LEFT_MOTOR_ID, Constants.RIO_CAN_BUS);
    rightMotor = new TalonFX(FLYWHEEL_RIGHT_MOTOR_ID, Constants.RIO_CAN_BUS);

    // Configure motors
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;

    config.Feedback.VelocityFilterTimeConstant = 0.0;

    // PID configuration
    config.Slot0.kP = FLYWHEEL_GAINS.kP();
    config.Slot0.kI = FLYWHEEL_GAINS.kI();
    config.Slot0.kD = FLYWHEEL_GAINS.kD();
    config.Slot0.kS = FLYWHEEL_GAINS.kS();
    config.Slot0.kV = FLYWHEEL_GAINS.kV();
    config.Slot0.kA = FLYWHEEL_GAINS.kA();

    config.Slot1.kP = FLYWHEEL_RECOVERY_GAINS.kP();
    config.Slot1.kI = FLYWHEEL_RECOVERY_GAINS.kI();
    config.Slot1.kD = FLYWHEEL_RECOVERY_GAINS.kD();
    config.Slot1.kS = FLYWHEEL_RECOVERY_GAINS.kS();
    config.Slot1.kV = FLYWHEEL_RECOVERY_GAINS.kV();
    config.Slot1.kA = FLYWHEEL_RECOVERY_GAINS.kA();

    config.MotionMagic.MotionMagicCruiseVelocity = FLYWHEEL_CONSTRAINTS.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = FLYWHEEL_CONSTRAINTS.maxAcceleration();
    config.MotionMagic.MotionMagicJerk = FLYWHEEL_CONSTRAINTS.kJerk();

    // Voltage compensation
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    config.TorqueCurrent.PeakForwardTorqueCurrent = STATOR_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -STATOR_CURRENT_LIMIT;

    // Apply to both motors
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));
    rightMotor.setControl(new Follower(FLYWHEEL_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));

    // Get status signals
    position = leftMotor.getPosition();
    velocity = leftMotor.getVelocity();
    velocitySetpoint = leftMotor.getClosedLoopReference();
    appliedVolts = List.of(leftMotor.getMotorVoltage(), rightMotor.getMotorVoltage());
    supplyCurrent = List.of(leftMotor.getSupplyCurrent(), rightMotor.getSupplyCurrent());
    torqueCurrent = List.of(leftMotor.getTorqueCurrent(), rightMotor.getTorqueCurrent());
    temperature = List.of(leftMotor.getDeviceTemp(), rightMotor.getDeviceTemp());

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(250, velocity, velocitySetpoint);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        appliedVolts.get(0),
        appliedVolts.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        temperature.get(0),
        temperature.get(1));

    PhoenixUtil.tryUntilOk(
        5, () -> ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor));

    PhoenixUtil.registerSignals(
        FlywheelConstants.FLYWHEEL_CANBUS,
        position,
        appliedVolts.get(0),
        appliedVolts.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        temperature.get(0),
        temperature.get(1));

    gainNotifier =
        new Notifier(
            () -> {
              synchronized (signalLock) {
                BaseStatusSignal.refreshAll(velocity, velocitySetpoint);
                measuredVelocityRps = velocity.getValueAsDouble();
              }

              final double error = targetVelocityRps - measuredVelocityRps;

              final double enterRecovery = VELOCITY_GAIN_TOLERANCE_RPS;
              final double exitRecovery = VELOCITY_GAIN_TOLERANCE_RPS * 0.5;
              final double recoveryBoundary = VELOCITY_GAIN_TOLERANCE_RPS * 7.5;

              int newSlot = activeSlot;
              boolean inRange =
                  MathUtil.isNear(targetVelocityRps, measuredVelocityRps, recoveryBoundary);
              if (inRange) {
                if (activeSlot == 0 && error > enterRecovery) {
                  newSlot = 1;
                } else if (activeSlot == 1 && error < exitRecovery) {
                  newSlot = 0;
                }
              } else if (activeSlot == 1) {
                newSlot = 0;
              }

              Logger.recordOutput("Shooter/Flywheel/ActiveSlot", activeSlot);
              Logger.recordOutput("Shooter/Flywheel/NewSlot", newSlot);

              if (newSlot != activeSlot) {
                synchronized (controlLock) {
                  activeSlot = newSlot;
                  velocityVoltage.Slot = activeSlot;
                  if (velocityMode) {
                    leftMotor.setControl(velocityVoltage.withVelocity(targetVelocityRps));
                  }
                }
              }
            });
    gainNotifier.startPeriodic(0.004);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.isAllGood(
            position,
            velocity,
            velocitySetpoint,
            appliedVolts.get(0),
            supplyCurrent.get(0),
            torqueCurrent.get(0),
            temperature.get(0));
    inputs.rightMotorConnected =
        BaseStatusSignal.isAllGood(
            position,
            velocity,
            appliedVolts.get(1),
            supplyCurrent.get(1),
            torqueCurrent.get(1),
            temperature.get(1));

    synchronized (signalLock) {
      inputs.velocityRps = velocity.getValueAsDouble();
      inputs.velocitySetpoint = velocitySetpoint.getValueAsDouble();
    }

    inputs.leadAppliedVolts = appliedVolts.get(0).getValueAsDouble();
    inputs.positionRots = position.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrent.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrent.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    inputs.tempCelsius = temperature.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
  }

  @Override
  public void setVelocity(double velocityRps) {
    targetVelocityRps = velocityRps;
    velocityMode = true;
    synchronized (controlLock) {
      leftMotor.setControl(velocityVoltage.withVelocity(velocityRps));
    }
  }

  @Override
  public void setVoltage(double volts) {
    velocityMode = false;
    synchronized (controlLock) {
      leftMotor.setControl(voltageOut.withOutput(volts));
    }
  }

  @Override
  public void setGains(Gains gains, int slotId) {
    if (slotId == 0) {
      config.Slot0.kP = gains.kP();
      config.Slot0.kI = gains.kI();
      config.Slot0.kD = gains.kD();
      config.Slot0.kS = gains.kS();
      config.Slot0.kV = gains.kV();
      config.Slot0.kA = gains.kA();
      PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config.Slot0));
      PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config.Slot0));
    } else {
      config.Slot1.kP = gains.kP();
      config.Slot1.kI = gains.kI();
      config.Slot1.kD = gains.kD();
      config.Slot1.kS = gains.kS();
      config.Slot1.kV = gains.kV();
      config.Slot1.kA = gains.kA();
      PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config.Slot1));
      PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config.Slot1));
    }
  }

  @Override
  public void setConstraints(Constraints constraints) {
    config.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    config.MotionMagic.MotionMagicJerk = constraints.kJerk();
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config.MotionMagic));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config.MotionMagic));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config.MotorOutput));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config.MotorOutput));
  }
}
