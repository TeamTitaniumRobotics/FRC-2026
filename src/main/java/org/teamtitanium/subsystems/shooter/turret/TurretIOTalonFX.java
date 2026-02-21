package org.teamtitanium.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Rotations;
import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFX turretMotor;
  private final CANcoder cancoder1;
  private final CANcoder cancoder2;

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  protected final StatusSignal<Double> targetSetpoint;

  private final StatusSignal<Angle> cancoder1Position;
  private final StatusSignal<Angle> cancoder2Position;

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TURRET_MOTOR_ID, Constants.RIO_CAN_BUS);
    cancoder1 = new CANcoder(TURRET_CANCODER_1_ID, Constants.RIO_CAN_BUS);
    cancoder2 = new CANcoder(TURRET_CANCODER_2_ID, Constants.RIO_CAN_BUS);

    // Configure motor output
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current current limits
    motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio
    motorConfig.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;

    // Soft limits
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // PID configuration
    motorConfig.Slot0.kP = TURRET_GAINS.kP();
    motorConfig.Slot0.kI = TURRET_GAINS.kI();
    motorConfig.Slot0.kD = TURRET_GAINS.kD();
    motorConfig.Slot0.kS = TURRET_GAINS.kS();
    motorConfig.Slot0.kV = TURRET_GAINS.kV();
    motorConfig.Slot0.kA = TURRET_GAINS.kA();

    // Motion Magic configuration
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = TURRET_CONSTRAINTS.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = TURRET_CONSTRAINTS.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = 0; // Set to 0 for trapezoidal profile

    // Voltage compensation
    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = -12.0;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorConfig));

    // Configure CANcoders
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> cancoder1.getConfigurator().apply(cancoderConfig));
    PhoenixUtil.tryUntilOk(5, () -> cancoder2.getConfigurator().apply(cancoderConfig));

    // Get status signals
    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVoltage = turretMotor.getMotorVoltage();
    supplyCurrent = turretMotor.getSupplyCurrent();
    torqueCurrent = turretMotor.getTorqueCurrent();
    temperature = turretMotor.getDeviceTemp();

    targetSetpoint = turretMotor.getClosedLoopReference();

    cancoder1Position = cancoder1.getAbsolutePosition();
    cancoder2Position = cancoder2.getAbsolutePosition();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, cancoder1Position, cancoder2Position, targetSetpoint);

    PhoenixUtil.tryUntilOk(
        5, () -> ParentDevice.optimizeBusUtilizationForAll(turretMotor, cancoder1, cancoder2));

    PhoenixUtil.registerSignals(
        Constants.RIO_CAN_BUS,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature,
        cancoder1Position,
        cancoder2Position,
        targetSetpoint);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();

    inputs.setpointRots = targetSetpoint.getValueAsDouble();

    inputs.cancoder1Connected = BaseStatusSignal.isAllGood(cancoder1Position);
    inputs.cancoder1PositionRots = cancoder1Position.getValueAsDouble();

    inputs.cancoder2Connected = BaseStatusSignal.isAllGood(cancoder2Position);
    inputs.cancoder2PositionRots = cancoder2Position.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRots) {
    turretMotor.setControl(motionMagicVoltage.withPosition(positionRots));
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setGains(Gains gains) {
    motorConfig.Slot0.kP = gains.kP();
    motorConfig.Slot0.kI = gains.kI();
    motorConfig.Slot0.kD = gains.kD();
    motorConfig.Slot0.kS = gains.kS();
    motorConfig.Slot0.kV = gains.kV();
    motorConfig.Slot0.kG = gains.kG();
    motorConfig.Slot0.kA = gains.kA();
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setConstraints(Constraints constraints) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotorPosition(double positionRots) {
    turretMotor.setPosition(positionRots);
  }
}
