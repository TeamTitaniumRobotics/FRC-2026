package org.teamtitanium.utils;

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
    private static List<VirtualSubsystem> virtualSubsystems = new ArrayList<>();

    public VirtualSubsystem() {
        virtualSubsystems.add(this);
    }

    public static void periodicAll() {
        for (VirtualSubsystem virtualSubsystem : virtualSubsystems) {
            virtualSubsystem.periodic();
        }
    }

    public abstract void periodic();
}
