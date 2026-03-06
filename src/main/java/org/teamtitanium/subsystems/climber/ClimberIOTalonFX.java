package org.teamtitanium.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

public class ClimberIOTalonFX implements ClimberIO {
  protected final TalonFX climberMotor;

  private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC =
      new MotionMagicTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Double> targetSetpoint;

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(CLIMBER_MOTOR_ID, CLIMBER_CAN_BUS);

    motorConfig.MotorOutput.Inverted =
        CLIMBER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.StatorCurrentLimit = CLIMBER_STATOR_CURRENT_LIMIT;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = CLIMBER_SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = CLIMBER_STATOR_CURRENT_LIMIT;
    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -CLIMBER_STATOR_CURRENT_LIMIT;

    motorConfig.Feedback.SensorToMechanismRatio = CLIMBER_GEAR_RATIO;

    motorConfig.Slot0.kP = CLIMBER_GAINS_SLOT_0.kP();
    motorConfig.Slot0.kI = CLIMBER_GAINS_SLOT_0.kI();
    motorConfig.Slot0.kD = CLIMBER_GAINS_SLOT_0.kD();
    motorConfig.Slot0.kS = CLIMBER_GAINS_SLOT_0.kS();
    motorConfig.Slot0.kV = CLIMBER_GAINS_SLOT_0.kV();
    motorConfig.Slot0.kG = CLIMBER_GAINS_SLOT_0.kG();
    motorConfig.Slot0.kA = CLIMBER_GAINS_SLOT_0.kA();

    motorConfig.Slot1.kP = CLIMBER_GAINS_SLOT_1.kP();
    motorConfig.Slot1.kI = CLIMBER_GAINS_SLOT_1.kI();
    motorConfig.Slot1.kD = CLIMBER_GAINS_SLOT_1.kD();
    motorConfig.Slot1.kS = CLIMBER_GAINS_SLOT_1.kS();
    motorConfig.Slot1.kV = CLIMBER_GAINS_SLOT_1.kV();
    motorConfig.Slot1.kG = CLIMBER_GAINS_SLOT_1.kG();
    motorConfig.Slot1.kA = CLIMBER_GAINS_SLOT_1.kA();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = CLIMBER_CONSTRAINTS.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = CLIMBER_CONSTRAINTS.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = CLIMBER_CONSTRAINTS.kJerk();

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        metersToMotorRotations(CLIMBER_MAX_EXTENSION.in(Meters));
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        metersToMotorRotations(CLIMBER_MIN_EXTENSION.in(Meters));

    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(motorConfig));

    position = climberMotor.getPosition();
    velocity = climberMotor.getVelocity();
    appliedVoltage = climberMotor.getMotorVoltage();
    supplyCurrent = climberMotor.getSupplyCurrent();
    torqueCurrent = climberMotor.getTorqueCurrent();
    temperature = climberMotor.getDeviceTemp();
    targetSetpoint = climberMotor.getClosedLoopReference();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    BaseStatusSignal.setUpdateFrequencyForAll(50, targetSetpoint);

    PhoenixUtil.registerSignals(
        CLIMBER_CAN_BUS,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature,
        targetSetpoint);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
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
  public void setPosition(double positionRots, int slotId) {
    climberMotor.setControl(
        motionMagicTorqueCurrentFOC.withPosition(positionRots).withSlot(slotId));
  }

  @Override
  public void setCurrent(double amps) {
    climberMotor.setControl(torqueCurrentFOC.withOutput(amps));
  }

  @Override
  public void setGains(Gains gains, int slotId) {
    switch (slotId) {
      case 0 -> {
        motorConfig.Slot0.kP = gains.kP();
        motorConfig.Slot0.kI = gains.kI();
        motorConfig.Slot0.kD = gains.kD();
        motorConfig.Slot0.kS = gains.kS();
        motorConfig.Slot0.kV = gains.kV();
        motorConfig.Slot0.kG = gains.kG();
        motorConfig.Slot0.kA = gains.kA();
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(motorConfig.Slot0));
      }
      case 1 -> {
        motorConfig.Slot1.kP = gains.kP();
        motorConfig.Slot1.kI = gains.kI();
        motorConfig.Slot1.kD = gains.kD();
        motorConfig.Slot1.kS = gains.kS();
        motorConfig.Slot1.kV = gains.kV();
        motorConfig.Slot1.kG = gains.kG();
        motorConfig.Slot1.kA = gains.kA();
        PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(motorConfig.Slot1));
      }
    }
  }

  @Override
  public void setConstraints(Constraints constraints) {
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    motorConfig.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    motorConfig.MotionMagic.MotionMagicJerk = constraints.kJerk();
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    motorConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> climberMotor.getConfigurator().apply(motorConfig));
  }

  @Override
  public void setMotorPosition(double positionRots) {
    climberMotor.setPosition(positionRots);
  }

  @Override
  public void stop() {
    climberMotor.stopMotor();
  }
}
