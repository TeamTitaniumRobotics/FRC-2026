package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.Robot;
import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.Constants.Gains;
import org.teamtitanium.utils.LoggedTracer;
import org.teamtitanium.utils.LoggedTunableNumber;
import org.teamtitanium.utils.TunerConstants;

/** Represents a single swerve module, containing both drive and turn motors. */
public class SwerveModule {
  private static final LoggedTunableNumber drivekP =
      new LoggedTunableNumber("Swerve/Module/DrivekP");
  private static final LoggedTunableNumber drivekD =
      new LoggedTunableNumber("Swerve/Module/DrivekD");
  private static final LoggedTunableNumber drivekS =
      new LoggedTunableNumber("Swerve/Module/DrivekS");
  private static final LoggedTunableNumber drivekV =
      new LoggedTunableNumber("Swerve/Module/DrivekV");
  private static final LoggedTunableNumber drivekA =
      new LoggedTunableNumber("Swerve/Module/DrivekA");
  private static final LoggedTunableNumber drivekT =
      new LoggedTunableNumber("Swerve/Module/DrivekT");
  private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Swerve/Module/TurnkP");
  private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Swerve/Module/TurnkD");

  static {
    switch (Constants.getMode()) {
      case REAL:
        drivekP.initDefault(TunerConstants.FrontLeft.DriveMotorGains.kP);
        drivekD.initDefault(TunerConstants.FrontLeft.DriveMotorGains.kD);
        drivekS.initDefault(TunerConstants.FrontLeft.DriveMotorGains.kS);
        drivekV.initDefault(TunerConstants.FrontLeft.DriveMotorGains.kV);
        drivekA.initDefault(TunerConstants.FrontLeft.DriveMotorGains.kA);
        drivekT.initDefault(
            TunerConstants.FrontLeft.DriveMotorGearRatio / DCMotor.getKrakenX60(1).KtNMPerAmp);
        turnkP.initDefault(TunerConstants.FrontLeft.SteerMotorGains.kP);
        turnkD.initDefault(TunerConstants.FrontLeft.SteerMotorGains.kD);
        break;
      default:
        drivekP.initDefault(0.1);
        drivekD.initDefault(0.0);
        drivekS.initDefault(0.014);
        drivekV.initDefault(0.134);
        drivekA.initDefault(0.02);
        drivekT.initDefault(0.0);
        turnkP.initDefault(10.0);
        turnkD.initDefault(0.0);
        break;
    }
  }

  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  // Alerts for connection issues
  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnCANcoderDisconnectedAlert;

  // Debouncers for connection alerts
  private final Debouncer driveDisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnDisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnCANcoderDisconnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public SwerveModule(
      SwerveModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    // Initialize alerts
    driveDisconnectedAlert =
        new Alert("Drive motor disconnected on module " + index, AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Turn motor disconnected on module " + index, AlertType.kError);
    turnCANcoderDisconnectedAlert =
        new Alert("Turn CANcoder disconnected on module " + index, AlertType.kError);
  }

  /** Updates the inputs for the module. */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
  }

  /** Periodic method for the module, updating gains and checking connections. */
  public void periodic() {
    // Update drive and turn gains if they have changed
    if (drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || drivekS.hasChanged(hashCode())
        || drivekV.hasChanged(hashCode())
        || drivekA.hasChanged(hashCode())) {
      io.setDriveGains(
          new Gains(
              drivekP.get(), 0.0, drivekD.get(), drivekS.get(), drivekV.get(), 0.0, drivekA.get()));
    }

    if (turnkP.hasChanged(hashCode()) || turnkD.hasChanged(hashCode())) {
      io.setTurnGains(new Gains(turnkP.get(), 0.0, turnkD.get()));
    }

    // Update odometry positions
    int sampleCount = inputs.odometryDrivePositionsRad.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      odometryPositions[i] =
          new SwerveModulePosition(positionMeters, inputs.odometryTurnPositions[i]);
    }

    // Update connection alerts with debouncing
    driveDisconnectedAlert.set(
        !driveDisconnectedDebouncer.calculate(
            inputs.moduleData.driveConnected() && !Robot.isInitializing()));
    turnDisconnectedAlert.set(
        !turnDisconnectedDebouncer.calculate(
            inputs.moduleData.turnConnected() && !Robot.isInitializing()));
    turnCANcoderDisconnectedAlert.set(
        !turnCANcoderDisconnectedDebouncer.calculate(
            inputs.moduleData.turnCANcoderConnected() && !Robot.isInitializing()));

    // Log tracer
    LoggedTracer.record("Swerve/Module" + index);
  }

  /**
   * Runs the module at the desired state.
   *
   * @param state The desired state of the module.
   */
  public void runSetpoint(SwerveModuleState state) {
    double speedRadPerSec = state.speedMetersPerSecond / constants.WheelRadius;
    io.setDriveVelocity(speedRadPerSec);
    io.setTurnPosition(state.angle.getRadians());
  }

  /**
   * Runs the module at the desired state with a specified torque FF.
   *
   * @param state The desired state of the module.
   * @param wheelTorqueNm The desired wheel torque FF in Newton-meters.
   */
  public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
    double speedRadPerSec = state.speedMetersPerSecond / constants.WheelRadius;
    io.setDriveVelocity(speedRadPerSec, wheelTorqueNm * drivekT.get());
    io.setTurnPosition(state.angle.getRadians());
  }

  /**
   * Runs the module for characterization.
   *
   * @param output The output to run the drive motor at.
   */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(0.0);
  }

  /** Stops the module, disabling both motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /**
   * Gets the current angle of the module.
   *
   * @return The current angle of the module.
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(inputs.moduleData.turnPositionRad());
  }

  /**
   * Gets the current position of the module in meters.
   *
   * @return The current position of the module in meters.
   */
  public double getPositionMeters() {
    return inputs.moduleData.drivePositionRad() * constants.WheelRadius;
  }

  /**
   * Gets the current velocity of the module in meters per second.
   *
   * @return The current velocity of the module in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return inputs.moduleData.driveVelocityRadPerSec() * constants.WheelRadius;
  }

  /**
   * Gets the current SwerveModulePosition of the module.
   *
   * @return The current SwerveModulePosition of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * Gets the current SwerveModuleState of the module.
   *
   * @return The current SwerveModuleState of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Gets the odometry positions of the module.
   *
   * @return The odometry positions of the module.
   */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /**
   * Gets the wheel radius characterization position in radians.
   *
   * @return The wheel radius characterization position in radians.
   */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.moduleData.drivePositionRad();
  }

  /**
   * Gets the feedforward characterization velocity in rotations per second.
   *
   * @return The feedforward characterization velocity in rotations per second.
   */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.moduleData.driveVelocityRadPerSec());
  }

  /**
   * Sets the brake mode of the module.
   *
   * @param brake True to enable brake mode, false to disable.
   */
  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
