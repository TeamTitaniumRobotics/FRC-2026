package org.teamtitanium.subsystems.swerve;

import org.teamtitanium.utils.PhoenixUtil;
import org.teamtitanium.utils.TunerConstants;

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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
    protected final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> swerveConstants;

    protected final TalonFX driveMotor;
    protected final TalonFX turnMotor;
    protected final CANcoder turnCANcoder;

    protected final VoltageOut voltageOut = new VoltageOut(0.0);
    protected final PositionVoltage positionVoltage = new PositionVoltage(0.0);
    protected final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    protected final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
    protected final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0);
    protected final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);

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

    private final Debouncer driveConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer turnConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer turnCanCoderConnectedDebouncer = new Debouncer(0.5);

    protected SwerveModuleIOTalonFX(SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> swerveConstants) {
        this.swerveConstants = swerveConstants;

        driveMotor = new TalonFX(swerveConstants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        turnMotor = new TalonFX(swerveConstants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
        turnCANcoder = new CANcoder(swerveConstants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

        var driveConfig = swerveConstants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = swerveConstants.DriveMotorGains;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = swerveConstants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -swerveConstants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = swerveConstants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = swerveConstants.DriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

        
    }
}
