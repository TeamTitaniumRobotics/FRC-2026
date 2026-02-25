package org.teamtitanium.utils.virtualsubsystem;

public interface IVirtualSubsystem {
  public default void periodic() {}

  public default void simulationPeriodic() {}
}
