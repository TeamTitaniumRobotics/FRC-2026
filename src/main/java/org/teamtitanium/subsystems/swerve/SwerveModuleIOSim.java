package org.teamtitanium.subsystems.swerve;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.teamtitanium.utils.PhoenixUtil;

public class SwerveModuleIOSim extends SwerveModuleIOTalonFX {
  private final SwerveModuleSimulation moduleSimulation;

  public SwerveModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          swerveConstants,
      SwerveModuleSimulation moduleSimulation) {
    super(swerveConstants);

    this.moduleSimulation = moduleSimulation;

    moduleSimulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveMotor));
    moduleSimulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnMotor, turnCANcoder));
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    super.updateInputs(inputs);

    inputs.odometryDrivePositionsRad =
        Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = moduleSimulation.getCachedSteerAbsolutePositions();
  }
}
