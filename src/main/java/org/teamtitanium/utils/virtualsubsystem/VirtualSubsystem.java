package org.teamtitanium.utils.virtualsubsystem;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem implements IVirtualSubsystem {
  private static List<VirtualSubsystem> virtualSubsystems = new ArrayList<>();

  public VirtualSubsystem() {
    virtualSubsystems.add(this);
  }

  public static void periodicAll() {
    for (VirtualSubsystem virtualSubsystem : virtualSubsystems) {
      virtualSubsystem.periodic();
    }
  }

  public static void simulationPeriodicAll() {
    for (VirtualSubsystem virtualSubsystem : virtualSubsystems) {
      virtualSubsystem.simulationPeriodic();
    }
  }
}
