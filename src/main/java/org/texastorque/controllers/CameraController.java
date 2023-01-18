package org.texastorque.controllers;

import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.texastorque.Field;
import org.texastorque.torquelib.sensors.TorqueVision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class CameraController {
    public final PhotonPoseEstimator photonPoseEstimator;
    public final TorqueVision camera;
    public static final String CAMERA_NAME = "torquevision";
    public static final Transform3d CAMERA_TO_CENTER = new Transform3d(
            new Translation3d(-Units.inchesToMeters(29 * .5), Units.inchesToMeters(19.75), -Units.inchesToMeters(2)),
            new Rotation3d());

    public List<AprilTag> getAprilTags() {
        return Field.APRIL_TAGS.entrySet().stream().map(entry -> new AprilTag(entry.getKey(), entry.getValue())).toList();
        
    }

    public CameraController() throws IOException {
        camera = new TorqueVision(CAMERA_NAME, CAMERA_TO_CENTER);

        // final boolean isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
        // final AprilTagFieldLayout layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        // layout.setOrigin(isRedAlliance ? OriginPosition.kRedAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);

        final AprilTagFieldLayout layout = new AprilTagFieldLayout(getAprilTags(), Field.FIELD_LENGTH, Field.FIELD_WIDTH);
        photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, camera.getPhotonCamera(), CAMERA_TO_CENTER);

    }

    public void update(final BiConsumer<Pose2d, Double> addVisionMeasurement) {    
        Optional<EstimatedRobotPose> optionalEstimatedPose = photonPoseEstimator.update();
        if(optionalEstimatedPose.isPresent()) {
            EstimatedRobotPose estimatedPose = optionalEstimatedPose.get();
            addVisionMeasurement.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        }
        
    }
}
