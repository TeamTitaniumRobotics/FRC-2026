package org.teamtitanium.subsystems.intake.rack;

import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
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

public class IntakeRackIOTalonFX implements IntakeRackIO {
  protected final TalonFX rackMotor;

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final DynamicMotionMagicVoltage motionMagicVoltage =
      new DynamicMotionMagicVoltage(
          0.0, RACK_CONSTRAINTS.maxVelocity(), RACK_CONSTRAINTS.maxAcceleration());
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Double> targetSetpoint;

  public IntakeRackIOTalonFX() {
    rackMotor = new TalonFX(RACK_MOTOR_ID, RACK_CAN_BUS);

    motorConfig.MotorOutput.Inverted =
        RACK_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = RACK_GEAR_RATIO;

    motorConfig.Slot0.kP = RACK_GAINS.kP();
    motorConfig.Slot0.kI = RACK_GAINS.kI();
    motorConfig.Slot0.kD = RACK_GAINS.kD();
    motorConfig.Slot0.kS = RACK_GAINS.kS();
    motorConfig.Slot0.kV = RACK_GAINS.kV();
    motorConfig.Slot0.kG = RACK_GAINS.kG();
    motorConfig.Slot0.kA = RACK_GAINS.kA();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = RACK_CONSTRAINTS.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = RACK_CONSTRAINTS.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = RACK_CONSTRAINTS.kJerk();

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        metersToMotorRotations(MAX_EXTENSION.in(Meters));
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        metersToMotorRotations(MIN_EXTENSION.in(Meters));

    motorConfig.Voltage.PeakForwardVoltage = 12.0;
    motorConfig.Voltage.PeakReverseVoltage = -12.0;

    PhoenixUtil.tryUntilOk(5, () -> rackMotor.getConfigurator().apply(motorConfig));

    position = rackMotor.getPosition();
    velocity = rackMotor.getVelocity();
    appliedVoltage = rackMotor.getMotorVoltage();
    supplyCurrent = rackMotor.getSupplyCurrent();
    torqueCurrent = rackMotor.getTorqueCurrent();
    temperature = rackMotor.getDeviceTemp();
    targetSetpoint = rackMotor.getClosedLoopReference();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    BaseStatusSignal.setUpdateFrequencyForAll(50, targetSetpoint);

    PhoenixUtil.registerSignals(
        RACK_CAN_BUS,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature,
        targetSetpoint);
  }

  @Override
  public void updateInputs(IntakeRackIOInputs inputs) {
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

    inputs.positionMeters = motorRotationsToMeters(inputs.positionRots);
    inputs.velocityMps = motorRotationsToMeters(inputs.velocityRps);
    inputs.setpointMeters = motorRotationsToMeters(inputs.setpointRots);
  }

  @Override
  public void setPosition(double positionRots) {
    rackMotor.setControl(
        motionMagicVoltage
            .withPosition(positionRots)
            .withVelocity(RACK_CONSTRAINTS.maxVelocity())
            .withAcceleration(RACK_CONSTRAINTS.maxAcceleration()));
  }

  @Override
  public void setPosition(double positionRots, Constraints constraints) {
    rackMotor.setControl(
        motionMagicVoltage
            .withPosition(positionRots)
            .withVelocity(constraints.maxVelocity())
            .withAcceleration(constraints.maxAcceleration()));
  }

  @Override
  public void setVoltage(double volts) {
    rackMotor.setControl(voltageOut.withOutput(volts));
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
    PhoenixUtil.tryUntilOk(5, () -> rackMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setConstraints(Constraints constraints) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = constraints.kJerk();
    PhoenixUtil.tryUntilOk(5, () -> rackMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> rackMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotorPosition(double positionRots) {
    rackMotor.setPosition(positionRots);
  }
}
