package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.PhoenixUtil;
import org.teamtitanium.utils.TunerConstants;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  protected final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      swerveConstants;

  protected final TalonFX driveMotor;
  protected final TalonFX turnMotor;
  protected final CANcoder turnCANcoder;

  protected final VoltageOut voltageOut = new VoltageOut(0.0);
  protected final PositionVoltage positionVoltage = new PositionVoltage(0.0);
  protected final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

  protected final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
  protected final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0);
  protected final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0);

  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  protected final StatusSignal<Angle> drivePosition;
  protected final StatusSignal<AngularVelocity> driveVelocity;
  protected final StatusSignal<Voltage> driveAppliedVolts;
  protected final StatusSignal<Current> driveSupplyCurrentAmps;
  protected final StatusSignal<Current> driveTorqueCurrentAmps;
  protected final StatusSignal<Temperature> driveTempCelcius;

  protected final StatusSignal<Angle> turnAbsolutePosition;
  protected final StatusSignal<Angle> turnPosition;
  protected final StatusSignal<AngularVelocity> turnVelocity;
  protected final StatusSignal<Voltage> turnAppliedVolts;
  protected final StatusSignal<Current> turnSupplyCurrentAmps;
  protected final StatusSignal<Current> turnTorqueCurrentAmps;
  protected final StatusSignal<Temperature> turnTempCelcius;

  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  protected SwerveModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          swerveConstants) {
    this.swerveConstants = swerveConstants;

    driveMotor =
        new TalonFX(swerveConstants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    turnMotor =
        new TalonFX(swerveConstants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    turnCANcoder =
        new CANcoder(swerveConstants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

    var driveConfig = swerveConstants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = swerveConstants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = swerveConstants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = swerveConstants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -swerveConstants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = swerveConstants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    driveConfig.MotorOutput.Inverted =
        swerveConstants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = swerveConstants.SteerMotorGains;
    if (Constants.getMode() == Constants.Mode.SIM) {
      turnConfig.Slot0.withKD(0.5).withKS(0);
    }
    turnConfig.Feedback.FeedbackRemoteSensorID = swerveConstants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (swerveConstants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> throw new RuntimeException("Unsupported swerve feedback source");
        };
    turnConfig.Feedback.RotorToSensorRatio = swerveConstants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / swerveConstants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.1;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * swerveConstants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        swerveConstants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

    var canCoderConfig = swerveConstants.EncoderInitialConfigs;
    canCoderConfig.MagnetSensor.MagnetOffset = swerveConstants.EncoderOffset;
    canCoderConfig.MagnetSensor.SensorDirection =
        swerveConstants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> turnCANcoder.getConfigurator().apply(canCoderConfig, 0.25));

    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition().clone());
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnMotor.getPosition().clone());

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
    driveTorqueCurrentAmps = driveMotor.getTorqueCurrent();
    driveTempCelcius = driveMotor.getDeviceTemp();

    turnAbsolutePosition = turnCANcoder.getAbsolutePosition();
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    turnSupplyCurrentAmps = turnMotor.getSupplyCurrent();
    turnTorqueCurrentAmps = turnMotor.getTorqueCurrent();
    turnTempCelcius = turnMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Swerve.ODOMETRY_FREQUENCY, drivePosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTempCelcius,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTempCelcius);
    PhoenixUtil.tryUntilOk(
        5, () -> ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor, turnCANcoder));

    PhoenixUtil.registerSignals(
        TunerConstants.kCANBus,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrentAmps,
        driveTorqueCurrentAmps,
        driveTempCelcius,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrentAmps,
        turnTorqueCurrentAmps,
        turnTempCelcius);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.swerveModuleData =
        new SwerveModuleIOData(
            BaseStatusSignal.isAllGood(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrentAmps,
                driveTorqueCurrentAmps,
                driveTempCelcius),
            Units.rotationsToRadians(drivePosition.getValueAsDouble()),
            Units.rotationsToRadians(driveVelocity.getValueAsDouble()),
            driveAppliedVolts.getValueAsDouble(),
            driveSupplyCurrentAmps.getValueAsDouble(),
            driveTorqueCurrentAmps.getValueAsDouble(),
            driveTempCelcius.getValueAsDouble(),
            BaseStatusSignal.isAllGood(
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrentAmps,
                turnTorqueCurrentAmps,
                turnTempCelcius),
            BaseStatusSignal.isAllGood(turnAbsolutePosition),
            turnAbsolutePosition.getValueAsDouble(),
            Units.rotationsToRadians(turnPosition.getValueAsDouble()),
            Units.rotationsToRadians(turnVelocity.getValueAsDouble()),
            turnAppliedVolts.getValueAsDouble(),
            turnSupplyCurrentAmps.getValueAsDouble(),
            turnTorqueCurrentAmps.getValueAsDouble(),
            turnTempCelcius.getValueAsDouble());

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setControl(
        switch (swerveConstants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageOut.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentFOC.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setControl(
        switch (swerveConstants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageOut.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentFOC.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double rotationsPerSec =
        Units.radiansToRotations(velocityRadPerSec)
            / swerveConstants.DriveMotorGearRatio; // TODO: verify you need to divide here
    driveMotor.setControl(
        switch (swerveConstants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageOut.withOutput(rotationsPerSec);
          case TorqueCurrentFOC -> torqueCurrentFOC.withOutput(rotationsPerSec);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec, double torqueFF) {
    double rotationsPerSec =
        Units.radiansToRotations(velocityRadPerSec)
            / swerveConstants.DriveMotorGearRatio; // TODO: verify you need to divide here
    driveMotor.setControl(
        switch (swerveConstants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltage.withVelocity(rotationsPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentFOC
              .withVelocity(rotationsPerSec)
              .withFeedForward(torqueFF);
        });
  }

  @Override
  public void setTurnPosition(double positionRad) {
    double rotations = Units.radiansToRotations(positionRad);
    turnMotor.setControl(
        switch (swerveConstants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltage.withPosition(rotations);
          case TorqueCurrentFOC -> positionTorqueCurrentFOC.withPosition(rotations);
        });
  }

  @Override
  public void setDriveGains(Gains gains) {
    var config = swerveConstants.DriveMotorInitialConfigs.Slot0;
    config
        .withKP(gains.kP())
        .withKI(gains.kI())
        .withKD(gains.kD())
        .withKS(gains.kS())
        .withKV(gains.kV())
        .withKA(gains.kA());
    PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(config));
  }

  @Override
  public void setTurnGains(Gains gains) {
    var config = swerveConstants.SteerMotorInitialConfigs.Slot0;
    config
        .withKP(gains.kP())
        .withKI(gains.kI())
        .withKD(gains.kD())
        .withKS(gains.kS())
        .withKV(gains.kV())
        .withKA(gains.kA());
    PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(config));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    brakeModeExecutor.execute(
        () -> {
          driveMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
          turnMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        });
  }
}
