package org.teamtitanium.subsystems.shooter.turret;

import static org.teamtitanium.subsystems.shooter.turret.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class TurretIOSim extends TurretIOTalonFX {
  private final DCMotorSim turretSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              TURRET_GEARBOX, TURRET_MOMENT_OF_INERTIA, TURRET_GEAR_RATIO),
          TURRET_GEARBOX);

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.cancoder1Connected = true;
    inputs.cancoder2Connected = true;

    var motorSimState = turretMotor.getSimState();
    motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Update simulation
    turretSim.setInputVoltage(motorSimState.getMotorVoltage());
    turretSim.update(0.02); // 20ms loop time

    // Update the real motor position for Motion Magic control
    motorSimState.setRawRotorPosition(turretSim.getAngularPositionRotations() * TURRET_GEAR_RATIO);
    motorSimState.setRotorVelocity(turretSim.getAngularVelocityRPM() / 60.0 * TURRET_GEAR_RATIO);

    super.updateInputs(inputs);

    Logger.recordOutput(
        "Turret/Sim/TurretPosition",
        new Pose3d(TURRET_TO_ROBOT.getTranslation(), Rotation3d.kZero)
            .plus(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(0.0, 0.0, Units.rotationsToRadians(inputs.positionRots)))));
    // new Pose3d(RobotState.getInstance().getEstimatedPose())
    //     // .plus(TURRET_TO_ROBOT)
    //     .plus(
    //         new Transform3d(
    //             Translation3d.kZero,
    //             new Rotation3d(0.0, 0.0, Units.rotationsToRadians(inputs.positionRots)))));
  }
}
