package org.teamtitanium.subsystems.shooter.flywheel;

import static org.teamtitanium.subsystems.shooter.flywheel.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.List;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;

public class FlywheelIOTalonFX implements FlywheelIO {
  protected final TalonFX leftMotor;
  protected final TalonFX rightMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVelocityVoltage motionMagicVelocityVoltage =
      new MotionMagicVelocityVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final List<StatusSignal<Voltage>> appliedVolts;
  private final List<StatusSignal<Current>> supplyCurrent;
  private final List<StatusSignal<Current>> torqueCurrent;
  private final List<StatusSignal<Temperature>> temperature;

  public FlywheelIOTalonFX() {
    leftMotor = new TalonFX(FLYWHEEL_LEFT_MOTOR_ID, Constants.RIO_CAN_BUS);
    rightMotor = new TalonFX(FLYWHEEL_RIGHT_MOTOR_ID, Constants.RIO_CAN_BUS);

    // Configure motors
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
    rightMotor.setControl(new Follower(FLYWHEEL_LEFT_MOTOR_ID, MotorAlignmentValue.Opposed));

    // Get status signals
    position = leftMotor.getPosition();
    velocity = leftMotor.getVelocity();
    appliedVolts = List.of(leftMotor.getMotorVoltage(), rightMotor.getMotorVoltage());
    supplyCurrent = List.of(leftMotor.getSupplyCurrent(), rightMotor.getSupplyCurrent());
    torqueCurrent = List.of(leftMotor.getTorqueCurrent(), rightMotor.getTorqueCurrent());
    temperature = List.of(leftMotor.getDeviceTemp(), rightMotor.getDeviceTemp());

    // Set update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        velocity,
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
        Constants.RIO_CAN_BUS,
        position,
        velocity,
        appliedVolts.get(0),
        appliedVolts.get(1),
        supplyCurrent.get(0),
        supplyCurrent.get(1),
        torqueCurrent.get(0),
        torqueCurrent.get(1),
        temperature.get(0),
        temperature.get(1));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.isAllGood(
            position,
            velocity,
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

    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRps = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrent.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    inputs.torqueCurrentAmps =
        torqueCurrent.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    inputs.tempCelsius = temperature.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
    temperature.stream().mapToDouble(s -> s.getValueAsDouble()).toArray();
  }

  @Override
  public void setVelocity(double velocityRps) {
    leftMotor.setControl(motionMagicVelocityVoltage.withVelocity(velocityRps));
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setControl(voltageOut.withOutput(volts));
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
}
