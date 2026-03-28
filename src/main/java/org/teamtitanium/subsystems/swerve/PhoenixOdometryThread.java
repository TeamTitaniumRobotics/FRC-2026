package org.teamtitanium.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.teamtitanium.utils.TunerConstants;

public class PhoenixOdometryThread extends Thread {
  private final Lock signalsLock = new ReentrantLock();
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static boolean isCANFD =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD();

  private static PhoenixOdometryThread instance;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  @Override
  public synchronized void start() {
    if (timestampQueues.size() > 0) {
      super.start();
    }
  }

  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    signalsLock.lock();
    Swerve.odometryLock.lock();

    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> registerSignal(DoubleSupplier supplier) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    signalsLock.lock();
    Swerve.odometryLock.lock();

    try {
      genericSignals.add(supplier);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    Swerve.odometryLock.lock();

    try {
      timestampQueues.add(queue);
    } finally {
      Swerve.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      BaseStatusSignal[] phoenixSignalsSnapshot;
      List<DoubleSupplier> genericSignalsSnapshot;
      List<Queue<Double>> phoenixQueuesSnapshot;
      List<Queue<Double>> genericQueuesSnapshot;
      List<Queue<Double>> timestampQueuesSnapshot;

      signalsLock.lock();
      try {
        phoenixSignalsSnapshot = phoenixSignals.clone();
        genericSignalsSnapshot = new ArrayList<>(genericSignals);
        phoenixQueuesSnapshot = new ArrayList<>(phoenixQueues);
        genericQueuesSnapshot = new ArrayList<>(genericQueues);
        timestampQueuesSnapshot = new ArrayList<>(timestampQueues);
      } finally {
        signalsLock.unlock();
      }

      try {
        if (isCANFD && phoenixSignalsSnapshot.length > 0) {
          BaseStatusSignal.waitForAll(2.0 / Swerve.ODOMETRY_FREQUENCY, phoenixSignalsSnapshot);
        } else {
          Thread.sleep((long) (1000.0 / Swerve.ODOMETRY_FREQUENCY));
          if (phoenixSignalsSnapshot.length > 0) {
            BaseStatusSignal.refreshAll(phoenixSignalsSnapshot);
          }
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return;
      }

      double timestamp = RobotController.getFPGATime() / 1e6;
      double totalLatency = 0.0;
      double[] phoenixValues = new double[phoenixSignalsSnapshot.length];
      for (int i = 0; i < phoenixSignalsSnapshot.length; i++) {
        BaseStatusSignal signal = phoenixSignalsSnapshot[i];
        totalLatency += signal.getTimestamp().getLatency();
        phoenixValues[i] = signal.getValueAsDouble();
      }

      if (phoenixSignalsSnapshot.length > 0) {
        timestamp -= totalLatency / phoenixSignalsSnapshot.length;
      }

      double[] genericValues = new double[genericSignalsSnapshot.size()];
      for (int i = 0; i < genericSignalsSnapshot.size(); i++) {
        genericValues[i] = genericSignalsSnapshot.get(i).getAsDouble();
      }

      Swerve.odometryLock.lock();

      try {
        for (int i = 0; i < phoenixValues.length; i++) {
          phoenixQueuesSnapshot.get(i).offer(phoenixValues[i]);
        }
        for (int i = 0; i < genericValues.length; i++) {
          genericQueuesSnapshot.get(i).offer(genericValues[i]);
        }
        for (int i = 0; i < timestampQueuesSnapshot.size(); i++) {
          timestampQueuesSnapshot.get(i).offer(timestamp);
        }
      } finally {
        Swerve.odometryLock.unlock();
      }
    }
  }
}
