// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.teamtitanium.utils;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/** Utility class to log the list of NetworkTables clients. */
public class NTClientLogger {
  private static final String tableName = "NTClients/";
  private static Set<String> lastRemoteIds = new HashSet<>();
  private static ByteBuffer intBuffer = ByteBuffer.allocate(4);

  private NTClientLogger() {}

  public static void log() {
    ConnectionInfo[] connections = NetworkTableInstance.getDefault().getConnections();
    Set<String> currentRemoteIds = new HashSet<>();

    // Log connected clients
    for (ConnectionInfo connection : connections) {
      lastRemoteIds.remove(connection.remote_id);
      currentRemoteIds.add(connection.remote_id);

      String clientTableName = tableName + connection.remote_id + "/";
      Logger.recordOutput(clientTableName + "Connected", true);
      Logger.recordOutput(clientTableName + "IPAddress", connection.remote_ip);
      Logger.recordOutput(clientTableName + "RemotePort", connection.remote_port);
      Logger.recordOutput(clientTableName + "LastUpdate", connection.last_update);

      intBuffer.rewind();
      Logger.recordOutput(
          clientTableName + "ProtocolVersion",
          intBuffer.putInt(connection.protocol_version).array());
    }

    // Log disconnected clients
    for (var remoteId : lastRemoteIds) {
      Logger.recordOutput(tableName + remoteId + "/Connected", false);
    }
    lastRemoteIds = currentRemoteIds;
  }
}
