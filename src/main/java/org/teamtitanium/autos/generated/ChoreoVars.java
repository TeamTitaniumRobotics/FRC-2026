package org.teamtitanium.autos.generated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final Distance FieldW = Units.Meters.of(8.069);
    public static final Distance TrenchW = Units.Meters.of(1.194);

    public static final class Poses {
        public static final Pose2d RFE = new Pose2d(7.936, 3.308, Rotation2d.fromRadians(1.571));
        public static final Pose2d RFS = new Pose2d(7.936, 0.895, Rotation2d.fromRadians(1.571));
        public static final Pose2d RTS = new Pose2d(4.399, 0.639, Rotation2d.kZero);

        private Poses() {}
    }

    private ChoreoVars() {}
}