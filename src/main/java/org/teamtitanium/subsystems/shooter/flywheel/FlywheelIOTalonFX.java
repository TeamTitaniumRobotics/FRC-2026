package org.teamtitanium.subsystems.shooter.flywheel;

import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

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
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  protected final TalonFX leftMotor;
  protected final TalonFX rightMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVoltage;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftTorqueCurrent;
  private final StatusSignal<Temperature> leftTemperature;

  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVoltage;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightTorqueCurrent;
  private final StatusSignal<Temperature> rightTemperature;

  public FlywheelIOTalonFX() {
    leftMotor = new TalonFX(FLYWHEEL_LEFT_MOTOR_ID, FLYWHEEL_CANBUS);
    rightMotor = new TalonFX(FLYWHEEL_RIGHT_MOTOR_ID, FLYWHEEL_CANBUS);

    // Configure motors
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast for flywheels

    // Current limits
    config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Gear ratio
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;

    // PID configuration
    config.Slot0.kP = FLYWHEEL_GAINS.kP();
    config.Slot0.kI = FLYWHEEL_GAINS.kI();
    config.Slot0.kD = FLYWHEEL_GAINS.kD();
    config.Slot0.kS = FLYWHEEL_GAINS.kS();
    config.Slot0.kV = FLYWHEEL_GAINS.kV();
    config.Slot0.kA = FLYWHEEL_GAINS.kA();

    // Voltage compensation
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    // Apply to both motors
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));

    // Right motor should be inverted (opposite side of shooter)
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));

    // Get status signals
    leftVelocity = leftMotor.getVelocity();
    leftAppliedVoltage = leftMotor.getMotorVoltage();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftTorqueCurrent = leftMotor.getTorqueCurrent();
    leftTemperature = leftMotor.getDeviceTemp();

    rightVelocity = rightMotor.getVelocity();
    rightAppliedVoltage = rightMotor.getMotorVoltage();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightTorqueCurrent = rightMotor.getTorqueCurrent();
    rightTemperature = rightMotor.getDeviceTemp();

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        leftVelocity,
        leftAppliedVoltage,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTemperature,
        rightVelocity,
        rightAppliedVoltage,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTemperature);

    PhoenixUtil.tryUntilOk(
        5, () -> ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor));

    PhoenixUtil.registerSignals(
        Constants.RIO_CAN_BUS,
        leftVelocity,
        leftAppliedVoltage,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTemperature,
        rightVelocity,
        rightAppliedVoltage,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTemperature);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftVelocity,
                leftAppliedVoltage,
                leftSupplyCurrent,
                leftTorqueCurrent,
                leftTemperature)
            .isOK();
    inputs.leftVelocityRps = leftVelocity.getValueAsDouble();
    inputs.leftAppliedVolts = leftAppliedVoltage.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTemperature.getValueAsDouble();

    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightVelocity,
                rightAppliedVoltage,
                rightSupplyCurrent,
                rightTorqueCurrent,
                rightTemperature)
            .isOK();
    inputs.rightVelocityRps = rightVelocity.getValueAsDouble();
    inputs.rightAppliedVolts = rightAppliedVoltage.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTemperature.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRps) {
    leftMotor.setControl(velocityControl.withVelocity(velocityRps));
    rightMotor.setControl(velocityControl.withVelocity(velocityRps));
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setControl(voltageOut.withOutput(volts));
    rightMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setGains(Gains gains) {
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
