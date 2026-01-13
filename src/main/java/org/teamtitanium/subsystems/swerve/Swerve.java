package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.teamtitanium.utils.Constants;
import org.teamtitanium.utils.PhoenixUtil;
import org.teamtitanium.utils.TunerConstants;

public class Swerve extends SubsystemBase {
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final Lock odometryLock = new ReentrantLock();

  @Override
  public void periodic() {
  }
}
