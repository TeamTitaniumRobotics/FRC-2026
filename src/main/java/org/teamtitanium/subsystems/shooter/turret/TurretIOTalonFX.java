package org.teamtitanium.subsystems.shooter.turret;

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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  private final StatusSignal<Angle> cancoder1Position;
  private final StatusSignal<Angle> cancoder2Position;

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TURRET_MOTOR_ID, TURRET_CANBUS);
    cancoder1 = new CANcoder(TURRET_CANCODER_1_ID, TURRET_CANBUS);
    cancoder2 = new CANcoder(TURRET_CANCODER_2_ID, TURRET_CANBUS);

    // Configure TalonFX
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio
    config.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;

    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE_ROTS;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE_ROTS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // PID configuration
    config.Slot0.kP = TURRET_GAINS.kP();
    config.Slot0.kI = TURRET_GAINS.kI();
    config.Slot0.kD = TURRET_GAINS.kD();
    config.Slot0.kS = TURRET_GAINS.kS();
    config.Slot0.kV = TURRET_GAINS.kV();
    config.Slot0.kG = TURRET_GAINS.kG();
    config.Slot0.kA = TURRET_GAINS.kA();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Motion Magic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = TURRET_MOTION_CONSTRAINTS.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = TURRET_MOTION_CONSTRAINTS.maxAcceleration();
    config.MotionMagic.MotionMagicJerk = 0; // Set to 0 for trapezoidal profile

    // Voltage compensation
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));

    // Configure CANcoders
    CANcoderConfiguration cancoderConfig1 = new CANcoderConfiguration();
    cancoderConfig1.MagnetSensor.SensorDirection =
        com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> cancoder1.getConfigurator().apply(cancoderConfig1));

    CANcoderConfiguration cancoderConfig2 = new CANcoderConfiguration();
    cancoderConfig2.MagnetSensor.SensorDirection =
        com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> cancoder2.getConfigurator().apply(cancoderConfig2));

    // Get status signals
    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVoltage = turretMotor.getMotorVoltage();
    supplyCurrent = turretMotor.getSupplyCurrent();
    torqueCurrent = turretMotor.getTorqueCurrent();
    temperature = turretMotor.getDeviceTemp();

    cancoder1Position = cancoder1.getAbsolutePosition();
    cancoder2Position = cancoder2.getAbsolutePosition();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    BaseStatusSignal.setUpdateFrequencyForAll(50, cancoder1Position, cancoder2Position);

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
        cancoder2Position);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature)
            .isOK();
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();

    inputs.cancoder1Connected = BaseStatusSignal.refreshAll(cancoder1Position).isOK();
    inputs.cancoder1PositionRots = cancoder1Position.getValueAsDouble() / CANCODER_1_RATIO;

    inputs.cancoder2Connected = BaseStatusSignal.refreshAll(cancoder2Position).isOK();
    inputs.cancoder2PositionRots = cancoder2Position.getValueAsDouble() / CANCODER_2_RATIO;
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
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kG = gains.kG();
    config.Slot0.kA = gains.kA();
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
  }

  @Override
  public void setConstraints(Constraints constraints) {
    config.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(config));
  }

  @Override
  public void setMotorPosition(double positionRots) {
    turretMotor.setPosition(positionRots);
  }

  @Override
  public void zeroWithCANcoders() {
    // Chinese Remainder Theorem for absolute positioning
    // This resolves the absolute position using two encoders with different gear ratios
    double pos1 = cancoder1Position.getValueAsDouble() / CANCODER_1_RATIO;
    double pos2 = cancoder2Position.getValueAsDouble() / CANCODER_2_RATIO;

    // Normalize positions to [0, 1)
    pos1 = pos1 - Math.floor(pos1);
    pos2 = pos2 - Math.floor(pos2);

    // Simple CRT implementation for two coprime moduli
    // This finds the unique position in the range determined by the two encoders
    double bestPosition = 0;
    double minError = Double.MAX_VALUE;

    // Search through possible positions
    for (int i = -10; i <= 10; i++) {
      double testPos = pos1 + i;
      double error = Math.abs((testPos * CANCODER_1_RATIO / CANCODER_2_RATIO) % 1.0 - pos2);
      error = Math.min(error, 1.0 - error); // Account for wraparound

      if (error < minError) {
        minError = error;
        bestPosition = testPos;
      }
    }

    // Clamp to mechanical limits
    bestPosition = Math.max(MIN_ANGLE_ROTS, Math.min(MAX_ANGLE_ROTS, bestPosition));

    setMotorPosition(bestPosition);
  }

  @Override
  public void stop() {
    turretMotor.stopMotor();
  }
}
