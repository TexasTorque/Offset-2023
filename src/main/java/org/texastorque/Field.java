package org.texastorque;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    public static final Pose3d reflectPositions(final Pose3d pose) {
        return new Pose3d(
            FIELD_LENGTH - pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            pose.getTranslation().getZ(),
            pose.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
    }

    private static final Map<Integer, Pose3d> reflectAprilTags() {
        final Map<Integer, Pose3d> map = Map.copyOf(APRIL_TAG_ORIGINALS);
        map.replaceAll((k, v) -> reflectPositions(v));
        return map;
    }

    public static final Map<Integer, Pose3d> APRIL_TAGS = 
        DriverStation.getAlliance() == DriverStation.Alliance.Blue
        ? APRIL_TAG_ORIGINALS
        : reflectAprilTags(); 


    public static double ALIGN_X_OFFSET_GRID = -(15.589758 - 14.72);
    public static double ALIGN_X_OFFSET_LOAD_ZONE = -1;

    public static enum AlignState {
        NONE, CENTER, RIGHT, LEFT;
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
        GRID_RIGHT(ALIGN_X_OFFSET_GRID, Units.inchesToMeters(22)), 
        GRID_LEFT(ALIGN_X_OFFSET_GRID, -Units.inchesToMeters(22)),
        LOAD_ZONE_RIGHT(ALIGN_X_OFFSET_LOAD_ZONE, -Units.inchesToMeters(30)),
        LOAD_ZONE_LEFT(ALIGN_X_OFFSET_LOAD_ZONE, Units.inchesToMeters(30));

        public Translation3d transl;

        private TranslationState(final double x, final double y) {
            transl = new Translation3d(x, y, 0);
        }

        public Pose2d calculate(final Pose3d pose) {
            return (new Pose3d(pose.getTranslation().plus(transl), pose.getRotation())).toPose2d();
        }
    }

    
}
