package org.teamtitanium.utils;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

public class CanivoreReader {
    private final Thread thread;
    private Optional<CANBusStatus> status = Optional.empty();

    public CanivoreReader(CANBus canBus) {
        thread = new Thread(() -> {
            while (true) {
                var statusTemp = Optional.of(canBus.getStatus());
                synchronized (this) {
                    status = statusTemp;
                }
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        thread.setName("CanivoreReader");
        thread.start();
    }

    public synchronized Optional<CANBusStatus> getStatus() {
        return status;
    }
}
