package org.teamtitanium.subsystems.genericroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;

public class GenericRollerIOTalonFX implements GenericRollerIO {
  private final TalonFX rollerMotor;

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVelocityVoltage motionMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  public GenericRollerIOTalonFX(
      int motorId,
      CANBus canbus,
      double reduction,
      Gains gains,
      Constraints constraints,
      double statorCurrentLimit,
      double supplyCurrentLimit,
      boolean inverted,
      boolean brake) {
    rollerMotor = new TalonFX(motorId, canbus);

    // Configure motor output
    motorConfig.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Current current limits
    motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = reduction;

    motorConfig.Slot0.kP = gains.kP();
    motorConfig.Slot0.kI = gains.kI();
    motorConfig.Slot0.kD = gains.kD();
    motorConfig.Slot0.kS = gains.kS();
    motorConfig.Slot0.kV = gains.kV();
    motorConfig.Slot0.kA = gains.kA();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = constraints.kJerk();

    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = -12.0;

    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(motorConfig));

    position = rollerMotor.getPosition();
    velocity = rollerMotor.getVelocity();
    appliedVoltage = rollerMotor.getMotorVoltage();
    supplyCurrent = rollerMotor.getSupplyCurrent();
    torqueCurrent = rollerMotor.getTorqueCurrent();
    temperature = rollerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(100, velocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, appliedVoltage, supplyCurrent, torqueCurrent, temperature);

    PhoenixUtil.registerSignals(
        canbus, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);

    rollerMotor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRps) {
    rollerMotor.setControl(motionMagicVelocityVoltage.withVelocity(velocityRps));
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }

  @Override
  public void setGains(Gains gains) {
    motorConfig.Slot0.kP = gains.kP();
    motorConfig.Slot0.kI = gains.kI();
    motorConfig.Slot0.kD = gains.kD();
    motorConfig.Slot0.kS = gains.kS();
    motorConfig.Slot0.kV = gains.kV();
    motorConfig.Slot0.kA = gains.kA();
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setConstraints(Constraints constraints) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = constraints.kJerk();
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(motorConfig));
  }
}
