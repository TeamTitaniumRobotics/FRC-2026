package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnCANcoderDisconnectedAlert;

  private final Debouncer driveDisconnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turnDisconnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turnCANcoderDisconnectedDebouncer = new Debouncer(0.5);

  public SwerveModule(
      SwerveModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    driveDisconnectedAlert =
        new Alert("Drive motor disconnected on module " + index, AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Turn motor disconnected on module " + index, AlertType.kError);
    turnCANcoderDisconnectedAlert =
        new Alert("Turn CANcoder disconnected on module " + index, AlertType.kError);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
  }

  public void periodic() {

    int sampleCount = inputs.odometryDrivePositionsRad.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      odometryPositions[i] =
          new SwerveModulePosition(positionMeters, inputs.odometryTurnPositions[i]);
    }
  }
}
