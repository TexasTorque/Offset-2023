/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Field {
    public static enum AprilTagType {
        GRID,
        LOAD_ZONE,
        INVALID;

        public boolean isGrid() { return this == GRID; }

        public boolean isLoadZone() { return this == LOAD_ZONE; }

        public boolean isValid() { return this != INVALID; }
    }
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);

    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    private static final Map<Integer, Pose3d> APRIL_TAG_ORIGINALS =
            Map.of(1,
                   new Pose3d(Units.inchesToMeters(610.77), // 15.589758
                              Units.inchesToMeters(42.19),  // 1.071626
                              Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI)),
                   2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI)), 3,
                   new Pose3d(Units.inchesToMeters(610.77),
                              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                              Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI)),
                   4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI)), 5,
                   new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d()), 6,
                   new Pose3d(Units.inchesToMeters(40.45),
                              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                              Units.inchesToMeters(18.22), new Rotation3d()),
                   7, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d()), 8,
                   new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d()));

    // Key (int): ID of the AprilTag
    // Value (boolean): True if the AprilTag is marks a grid, false if it marks
    // a load zone
    public static Map<Integer, Boolean> OUR_TAG_IDS = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? Map.of(8, true, 7, true, 6, true, 4, false)
                                                                                                                 : Map.of(1, true, 2, true, 3, true, 5, false);

    public static final Pose3d reflectPosition(final Pose3d pose) {
        return new Pose3d(FIELD_LENGTH - pose.getTranslation().getX(), FIELD_WIDTH - pose.getTranslation().getY(), pose.getTranslation().getZ(),
                          pose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
    }

    public static final Pose2d reflectPosition(final Pose2d pose) {
        return new Pose2d(FIELD_LENGTH - pose.getTranslation().getX(), FIELD_WIDTH - pose.getTranslation().getY(),
                          pose.getRotation().plus(Rotation2d.fromRadians(Math.PI)));
    }

    public static final Map<Integer, Pose3d> getAprilTagsMap() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? APRIL_TAG_ORIGINALS : reflectAprilTags();
    }

    // public static List<AprilTag> getAprilTagsList() {
    //     return getAprilTagsMap().entrySet().stream().map(entry -> new AprilTag(entry.getKey(), entry.getValue())).toList();
    // }

    //public static AprilTagFieldLayout getCurrentFieldLayout() { return new AprilTagFieldLayout(getAprilTagsList(), FIELD_LENGTH, FIELD_WIDTH); }

    public static AprilTagType getAprilTagType(final int id) {
        if (id <= 0 || id > 8) return AprilTagType.INVALID;
        if (id == 4 || id == 5) return AprilTagType.LOAD_ZONE;
        return AprilTagType.GRID;
    }

    private static final Map<Integer, Pose3d> reflectAprilTags() {
        final Map<Integer, Pose3d> newMap = new HashMap<>();
        for (final Map.Entry<Integer, Pose3d> aprilTag : APRIL_TAG_ORIGINALS.entrySet()) {
            newMap.put(aprilTag.getKey(), reflectPosition(aprilTag.getValue()));
        }
        return newMap;
    }
}
