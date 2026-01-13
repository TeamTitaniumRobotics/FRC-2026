package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.teamtitanium.utils.TunerConstants;

public class Swerve {
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final Lock odometryLock = new ReentrantLock();
}
