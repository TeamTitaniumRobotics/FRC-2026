package org.teamtitanium.subsystems.swerve;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Queue;
import org.teamtitanium.utils.TunerConstants;

public class GyroIOPigeon2 implements GyroIO {
  private Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

  private StatusSignal<Angle> yaw = pigeon.getYaw();
  private StatusSignal<Angle> pitch = pigeon.getPitch();
  private StatusSignal<Angle> roll = pigeon.getRoll();

  private StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityXWorld();
  private StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityYWorld();

  private Queue<Double> yawPositionQueue;
  private Queue<Double> yawTimestampQueue;

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw.setUpdateFrequency(Swerve.ODOMETRY_FREQUENCY);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);

    pigeon.optimizeBusUtilization();

    // PhoenixUtil.registerSignals(
    //     TunerConstants.kCANBus, yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, roll, pitch, yawVelocity, rollVelocity, pitchVelocity)
            .isOK();

    if (!inputs.connected && DriverStation.isDisabled()) {
      pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

      pigeon.getConfigurator().apply(new Pigeon2Configuration());
      pigeon.getConfigurator().setYaw(0.0);

      yaw = pigeon.getYaw();
      pitch = pigeon.getPitch();
      roll = pigeon.getRoll();
      yawVelocity = pigeon.getAngularVelocityZWorld();
      pitchVelocity = pigeon.getAngularVelocityXWorld();
      rollVelocity = pigeon.getAngularVelocityYWorld();

      yaw.setUpdateFrequency(Swerve.ODOMETRY_FREQUENCY);
      BaseStatusSignal.setUpdateFrequencyForAll(
          50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);

      pigeon.optimizeBusUtilization();

      // PhoenixUtil.registerSignals(
      //     TunerConstants.kCANBus, yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);

      yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    inputs.yawPositionRads = yaw.getValue().in(Radians);
    inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond);
    inputs.pitchPositionRads = pitch.getValue().in(Radians);
    inputs.pitchVelocityRadPerSec = pitchVelocity.getValue().in(RadiansPerSecond);
    inputs.rollPositionRads = roll.getValue().in(Radians);
    inputs.rollVelocityRadPerSec = rollVelocity.getValue().in(RadiansPerSecond);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositionsRads =
        yawPositionQueue.stream()
            .mapToDouble((Double value) -> Units.degreesToRadians(value))
            .toArray();
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void setGyroAngle(Angle angle) {
    pigeon.setYaw(angle);
  }
}
