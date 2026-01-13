package org.teamtitanium.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.teamtitanium.utils.PhoenixUtil;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.gyroData =
        new GyroIOData(
            true,
            gyroSimulation.getGyroReading().getRadians(),
            gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond),
            0,
            0,
            0,
            0);

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimestamps();

    Rotation2d[] cachedGyroReadingsRotation2d = gyroSimulation.getCachedGyroReadings();
    double[] cachedGyroReadingsRadians = new double[cachedGyroReadingsRotation2d.length];
    for (int i = 0; i < cachedGyroReadingsRotation2d.length; i++) {
      cachedGyroReadingsRadians[i] = cachedGyroReadingsRotation2d[i].getRadians();
    }
    inputs.odometryYawPositionsRads = cachedGyroReadingsRadians;
  }
}
