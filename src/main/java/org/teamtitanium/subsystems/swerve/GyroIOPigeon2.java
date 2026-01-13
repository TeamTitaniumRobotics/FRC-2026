package org.teamtitanium.subsystems.swerve;

import java.util.Queue;

import org.teamtitanium.utils.PhoenixUtil;
import org.teamtitanium.utils.TunerConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll = pigeon.getRoll();

    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityXWorld();
    private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityYWorld();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(Swerve.ODOMETRY_FREQUENCY);
        BaseStatusSignal.setUpdateFrequencyForAll(50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
        pigeon.optimizeBusUtilization();
        PhoenixUtil.registerSignals(TunerConstants.kCANBus, yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.gyroData = new GyroIOData(
            BaseStatusSignal.isAllGood(yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity), 
            Units.degreesToRadians(yaw.getValueAsDouble()),
            Units.degreesToRadians(yawVelocity.getValueAsDouble()),
            Units.degreesToRadians(pitch.getValueAsDouble()),
            Units.degreesToRadians(pitchVelocity.getValueAsDouble()),
            Units.degreesToRadians(roll.getValueAsDouble()),
            Units.degreesToRadians(rollVelocity.getValueAsDouble()));

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositionsRads = yawPositionQueue.stream().mapToDouble((Double value) -> Units.degreesToRadians(value)).toArray();
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}