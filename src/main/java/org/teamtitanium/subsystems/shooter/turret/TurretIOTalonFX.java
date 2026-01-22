package org.teamtitanium.subsystems.shooter.turret;

import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  public TurretIOTalonFX() {
    turretMotor = new TalonFX(TURRET_MOTOR_ID, TURRET_CANBUS);

    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVoltage = turretMotor.getMotorVoltage();
    supplyCurrent = turretMotor.getSupplyCurrent();
    torqueCurrent = turretMotor.getTorqueCurrent();
    temperature = turretMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temperature);
    PhoenixUtil.tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(turretMotor));

    PhoenixUtil.registerSignals(
        Constants.RIO_CAN_BUS,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }
}
