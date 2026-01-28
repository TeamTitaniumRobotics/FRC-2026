package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035;
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;

  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_MOTOR = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveMotorSim;
  private final DCMotorSim turnMotorSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;

  private PIDController driveController = new PIDController(DRIVE_KP, 0.0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0.0, TURN_KD);

  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public SwerveModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    driveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_MOTOR, constants.DriveInertia, constants.DriveMotorGearRatio),
            DRIVE_MOTOR);
    turnMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_MOTOR, constants.SteerInertia, constants.SteerMotorGearRatio),
            TURN_MOTOR);

    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveMotorSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }

    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnMotorSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));

    driveMotorSim.update(0.02);
    turnMotorSim.update(0.02);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveMotorSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveMotorSim.getCurrentDrawAmps());
    inputs.driveTorqueCurrentAmps = Math.abs(driveMotorSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnCANcoderConnected = true;
    inputs.turnAbsolutePositionRad = turnMotorSim.getAngularPositionRad();
    inputs.turnPositionRad = turnMotorSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnMotorSim.getCurrentDrawAmps());
    inputs.turnTorqueCurrentAmps = Math.abs(turnMotorSim.getCurrentDrawAmps());

    // inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {driveMotorSim.getAngularPositionRad()};
    inputs.odometryTurnPositions =
        new Rotation2d[] {new Rotation2d(turnMotorSim.getAngularPositionRad())};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocity) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocity) + DRIVE_KV * velocity;
    driveController.setSetpoint(velocity);
  }

  @Override
  public void setTurnPosition(double angle) {
    turnClosedLoop = true;
    turnController.setSetpoint(angle);
  }
}
