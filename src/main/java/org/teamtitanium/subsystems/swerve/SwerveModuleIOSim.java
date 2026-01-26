package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.teamtitanium.utils.TunerConstants;

public class SwerveModuleIOSim extends SwerveModuleIOTalonFX {
  private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.025, TunerConstants.FrontLeft.DriveMotorGearRatio),
          DCMotor.getKrakenX60(1));
  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.004, TunerConstants.FrontLeft.SteerMotorGearRatio),
          DCMotor.getKrakenX60(1));

  public SwerveModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          swerveConstants) {
    super(swerveConstants);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    var driveMotorSim = driveMotor.getSimState();

    driveMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    driveSim.setInputVoltage(driveMotorSim.getMotorVoltage());
    driveSim.update(0.02);

    driveMotorSim.setRawRotorPosition(
        driveSim.getAngularPosition().times(TunerConstants.FrontLeft.DriveMotorGearRatio));
    driveMotorSim.setRotorVelocity(
        driveSim.getAngularVelocity().times(TunerConstants.FrontLeft.DriveMotorGearRatio));

    var turnMotorSim = turnMotor.getSimState();

    turnMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    turnSim.setInputVoltage(turnMotorSim.getMotorVoltage());
    turnSim.update(0.02);

    turnMotorSim.setRawRotorPosition(
        turnSim.getAngularPosition().times(TunerConstants.FrontLeft.SteerMotorGearRatio));
    turnMotorSim.setRotorVelocity(
        turnSim.getAngularVelocity().times(TunerConstants.FrontLeft.SteerMotorGearRatio));

    super.updateInputs(inputs);

    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRadians(inputs.turnPositionRad)};
  }
}
