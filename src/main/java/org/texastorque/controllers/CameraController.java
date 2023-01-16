package org.texastorque.controllers;

import java.util.Optional;
import java.util.function.BiConsumer;

import org.texastorque.Field;
import org.texastorque.torquelib.sensors.TorqueVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class CameraController {

    public final TorqueVision camera;
    public static final String CAMERA_NAME = "torquevision";
    public static final Transform3d CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(-Units.inchesToMeters(29 * .5), Units.inchesToMeters(19.75), 0),
            new Rotation3d());

    public static final double TARGET_AMBIGUITY = 0.2;
    private double lastTimestamp = 0;

    public CameraController() {
        camera = new TorqueVision(CAMERA_NAME, CAMERA_TO_CENTER);
    }

    public void update(final BiConsumer<Pose2d, Double> addVisionMeasurement) {
        camera.update();
    
        final double timestamp = camera.getTimestamp();

        if (timestamp != lastTimestamp && camera.hasTargets()) {
            final Optional<Pose3d> estimatedPose = camera.getRobotPoseAprilTag3d(Field.APRIL_TAGS, TARGET_AMBIGUITY);
            if (estimatedPose.isPresent()) {
                final Pose2d pose = estimatedPose.get().toPose2d();            
                addVisionMeasurement.accept(pose, camera.getTimestamp());
            }
        }

        lastTimestamp = timestamp;
    }
}
