package org.teamtitanium.subsystems.shooter.hood;

import static org.teamtitanium.subsystems.shooter.hood.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Double> setpoint;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HOOD_MOTOR_ID, Constants.RIO_CAN_BUS);

    // Configure TalonFX
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio
    config.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;

    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE_ROTS;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE_ROTS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    // PID configuration
    config.Slot0.kP = HOOD_GAINS.kP();
    config.Slot0.kI = HOOD_GAINS.kI();
    config.Slot0.kD = HOOD_GAINS.kD();
    config.Slot0.kS = HOOD_GAINS.kS();
    config.Slot0.kV = HOOD_GAINS.kV();
    config.Slot0.kG = HOOD_GAINS.kG();
    config.Slot0.kA = HOOD_GAINS.kA();
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // Hood acts like an arm

    // Motion Magic configuration
    config.MotionMagic.MotionMagicCruiseVelocity = HOOD_MOTION_CONSTRAINTS.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = HOOD_MOTION_CONSTRAINTS.maxAcceleration();
    config.MotionMagic.MotionMagicJerk = 0; // Set to 0 for trapezoidal profile

    // Voltage compensation
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config));

    // Get status signals
    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    appliedVoltage = hoodMotor.getMotorVoltage();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    torqueCurrent = hoodMotor.getTorqueCurrent();
    temperature = hoodMotor.getDeviceTemp();
    setpoint = hoodMotor.getClosedLoopReference();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature,
        setpoint);
    PhoenixUtil.registerSignals(
        HoodConstants.HOOD_CANBUS,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature,
        setpoint);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            position,
            velocity,
            appliedVoltage,
            supplyCurrent,
            torqueCurrent,
            temperature,
            setpoint);
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
    inputs.setpointRots = setpoint.getValueAsDouble();
  }

  @Override
  public void setPosition(double positionRots) {
    hoodMotor.setControl(motionMagicVoltage.withPosition(positionRots));
  }

  @Override
  public void setVoltage(double volts) {
    hoodMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void stopMotor() {
    hoodMotor.stopMotor();
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
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config));
  }

  @Override
  public void setConstraints(Constraints constraints) {
    config.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(config));
  }

  @Override
  public void setMotorPosition(double positionRots) {
    hoodMotor.setPosition(positionRots);
  }
}
