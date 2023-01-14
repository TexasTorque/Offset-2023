package org.texastorque;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    private static final Map<Integer, Pose3d> APRIL_TAG_ORIGINALS =
    Map.of(
        1,
        new Pose3d(
            Units.inchesToMeters(610.77), // 15.589758
            Units.inchesToMeters(42.19), // 1.071626
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        2,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        3,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d(0.0, 0.0, Math.PI)),
        4,
        new Pose3d(
            Units.inchesToMeters(636.96),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d(0.0, 0.0, Math.PI)),
        5,
        new Pose3d(
            Units.inchesToMeters(14.25),
            Units.inchesToMeters(265.74),
            Units.inchesToMeters(27.38),
            new Rotation3d()),
        6,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        7,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(108.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()),
        8,
        new Pose3d(
            Units.inchesToMeters(40.45),
            Units.inchesToMeters(42.19),
            Units.inchesToMeters(18.22),
            new Rotation3d()));  

    public static final Pose3d reflectPosition(final Pose3d pose) {
        return new Pose3d(
            FIELD_LENGTH - pose.getTranslation().getX(),
            FIELD_WIDTH - pose.getTranslation().getY(),
            pose.getTranslation().getZ(),
            pose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
    }

   public static final Pose2d reflectPosition(final Pose2d pose) {
        return new Pose2d(
            FIELD_LENGTH - pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            pose.getRotation().plus(new Rotation2d(Math.PI)));
    }

    public static Map<Integer, Boolean> OUR_TAG_IDS = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? 
           Map.of(8, true, 7, true, 6, true, 4, false) : Map.of(1, true, 2, true, 3, true, 5, false);

    private static final Map<Integer, Pose3d> reflectAprilTags() {
        final Map<Integer, Pose3d> newMap = new HashMap<>();
        for (final Map.Entry<Integer, Pose3d> aprilTag : APRIL_TAG_ORIGINALS.entrySet()) {
            newMap.put(aprilTag.getKey(), reflectPosition(aprilTag.getValue()));
        }
        return newMap;
    }

    public static final Map<Integer, Pose3d> APRIL_TAGS = 
        DriverStation.getAlliance() == DriverStation.Alliance.Blue
        ? APRIL_TAG_ORIGINALS
        : reflectAprilTags(); 

    public static double ALIGN_X_OFFSET_GRID = (15.589758 - 14.72);
    public static double ALIGN_X_OFFSET_LOAD_ZONE = 1;

    public static enum AlignState {
        NONE, CENTER, RIGHT, LEFT;
    }

    public static enum GridState {
        NONE(-1, -1), LEFT(1, 6), CENTER(2, 7), RIGHT(3, 8);

        private final int blueID, redID;

        private GridState(final int blueID, final int redID) {
            this.redID = redID;
            this.blueID = blueID;
        }

        public int get() {
            return 1;
        }
    }

    public static enum AprilTagType {
        GRID, LOAD_ZONE, INVALID;

        public boolean isGrid() {
            return this == GRID;
        }

        public boolean isLoadZone() {
            return this == LOAD_ZONE;
        }

        public boolean isValid() {
            return this != INVALID;
        }
    }

    public static AprilTagType getAprilTagType(final int id) {
        if (id <= 0 || id > 8) return AprilTagType.INVALID;
        if (id == 4 || id == 5) return AprilTagType.LOAD_ZONE;
        return AprilTagType.GRID;
    }

    public static enum TranslationState {
        NONE(0, 0),
        GRID_CENTER(ALIGN_X_OFFSET_GRID, 0), 
        GRID_RIGHT(ALIGN_X_OFFSET_GRID, -Units.inchesToMeters(22)), 
        GRID_LEFT(ALIGN_X_OFFSET_GRID, Units.inchesToMeters(22)),
        LOAD_ZONE_RIGHT(ALIGN_X_OFFSET_LOAD_ZONE, -Units.inchesToMeters(30)),
        LOAD_ZONE_LEFT(ALIGN_X_OFFSET_LOAD_ZONE, Units.inchesToMeters(30));

        public Translation3d transl;
        private double x, y;

        private TranslationState(final double x, final double y) {
            this.x = x;
            this.y = y;
        }

        public Pose2d calculate(final Pose3d pose) {
            final Translation3d transl = new Translation3d(x * (pose.getX() > FIELD_LENGTH / 2 ? -1 : 1), y, 0);
            return (new Pose3d(pose.getTranslation().plus(transl), pose.getRotation())).toPose2d();
        }
    }

    
}
