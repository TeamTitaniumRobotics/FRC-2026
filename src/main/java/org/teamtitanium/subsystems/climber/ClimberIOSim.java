package org.teamtitanium.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.subsystems.climber.ClimberConstants.*;
import static org.teamtitanium.subsystems.intake.IntakeConstants.RackConstants.MAX_EXTENSION;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim extends ClimberIOTalonFX {
  private final ElevatorSim climberSim =
      new ElevatorSim(
          LinearSystemId.createElevatorSystem(
              CLIMBER_GEARBOX,
              Units.lbsToKilograms(150.0),
              SPROCKET_PD_METERS / 2.0,
              CLIMBER_GEAR_RATIO),
          CLIMBER_GEARBOX,
          0,
          MAX_EXTENSION.in(Meters),
          false,
          0);

  private final Notifier simNotifier;
  private double lastSimTime = 0.0;

  public ClimberIOSim() {
    super();

    var motorSimState = climberMotor.getSimState();

    lastSimTime = Utils.getCurrentTimeSeconds();

    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

              climberSim.setInputVoltage(motorSimState.getMotorVoltage());
              climberSim.update(deltaTime);

              motorSimState.setRawRotorPosition(
                  metersToMechRotations(climberSim.getPositionMeters()) * CLIMBER_GEAR_RATIO);
              motorSimState.setRotorVelocity(
                  metersToMechRotations(climberSim.getVelocityMetersPerSecond())
                      * CLIMBER_GEAR_RATIO);
            });
    simNotifier.startPeriodic(0.02);
  }
}
