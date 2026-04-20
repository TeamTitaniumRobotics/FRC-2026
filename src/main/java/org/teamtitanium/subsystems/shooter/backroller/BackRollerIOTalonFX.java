package org.teamtitanium.subsystems.shooter.backroller;

import static org.teamtitanium.subsystems.shooter.backroller.BackRollerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.teamtitanium.utils.Constants.Constraints;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;

public class BackRollerIOTalonFX implements BackRollerIO {
  protected final TalonFX rollerMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityVoltage velocityVoltage =
      new VelocityVoltage(0.0).withUpdateFreqHz(250.0).withEnableFOC(false);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Double> velocitySetpoint;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  public BackRollerIOTalonFX() {
    rollerMotor = new TalonFX(BACK_ROLLER_MOTOR_ID, BACK_ROLLER_CANBUS);

    // Configure motors
    config.MotorOutput.Inverted =
        BACK_ROLLER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio
    config.Feedback.SensorToMechanismRatio = BACK_ROLLER_GEAR_RATIO;

    config.Feedback.VelocityFilterTimeConstant = 0.0;

    // PID configuration
    config.Slot0.kP = BACK_ROLLER_GAINS.kP();
    config.Slot0.kI = BACK_ROLLER_GAINS.kI();
    config.Slot0.kD = BACK_ROLLER_GAINS.kD();
    config.Slot0.kS = BACK_ROLLER_GAINS.kS();
    config.Slot0.kV = BACK_ROLLER_GAINS.kV();
    config.Slot0.kA = BACK_ROLLER_GAINS.kA();

    config.MotionMagic.MotionMagicCruiseVelocity = BACK_ROLLER_CONSTRAINTS.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = BACK_ROLLER_CONSTRAINTS.maxAcceleration();
    config.MotionMagic.MotionMagicJerk = BACK_ROLLER_CONSTRAINTS.kJerk();

    // Voltage compensation
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    config.TorqueCurrent.PeakForwardTorqueCurrent = STATOR_CURRENT_LIMIT;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -STATOR_CURRENT_LIMIT;

    // Apply to both motors
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));

    // Get status signals
    velocity = rollerMotor.getVelocity();
    velocitySetpoint = rollerMotor.getClosedLoopReference();
    appliedVolts = rollerMotor.getMotorVoltage();
    supplyCurrent = rollerMotor.getSupplyCurrent();
    torqueCurrent = rollerMotor.getTorqueCurrent();
    temperature = rollerMotor.getDeviceTemp();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(250, velocity, velocitySetpoint);
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, appliedVolts, supplyCurrent, torqueCurrent, temperature);

    PhoenixUtil.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(rollerMotor));

    PhoenixUtil.registerSignals(
        BackRollerConstants.BACK_ROLLER_CANBUS,
        velocity,
        velocitySetpoint,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        temperature);
  }

  @Override
  public void updateInputs(BackRollerIOInputs inputs) {
    inputs.motorConnected =
        BaseStatusSignal.isAllGood(
            velocity, velocitySetpoint, appliedVolts, supplyCurrent, torqueCurrent, temperature);

    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.velocitySetpoint = velocitySetpoint.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRps) {
    rollerMotor.setControl(velocityVoltage.withVelocity(velocityRps));
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setGains(Gains gains) {
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();

    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));
  }

  @Override
  public void setConstraints(Constraints constraints) {
    config.MotionMagic.MotionMagicCruiseVelocity = constraints.maxVelocity();
    config.MotionMagic.MotionMagicAcceleration = constraints.maxAcceleration();
    config.MotionMagic.MotionMagicJerk = constraints.kJerk();

    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(config));
  }
}
