package org.texastorque;

import java.util.Map;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.25);
    public static final double FIELD_WIDTH = Units.inchesToMeters(315.5);

    private static final Map<Integer, Pose3d> APRIL_TAG_ORIGINALS =
    Map.of(
        1,
        new Pose3d(
            Units.inchesToMeters(610.77),
            Units.inchesToMeters(42.19),
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
}
