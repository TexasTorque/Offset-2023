package org.texastorque.toast.lib.apriltags;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.texastorque.toast.lib.apriltags.spoofer.AprilTagCameraSpoofer;
// import org.texastorque.toast.lib.apriltags.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class AprilTagPoseAggregator {
    private static final Map<String, PhotonPoseEstimator> estimators = new HashMap<>();
    public final SwerveDrivePoseEstimator estimator;
    private AprilTagFieldLayout layout;

    public static final void register(final String id, final AprilTagCameraSpoofer camera, final Transform3d centerToCamera) {
        final PhotonPoseEstimator ppe = new PhotonPoseEstimator(null, PoseStrategy.MULTI_TAG_PNP, camera, centerToCamera);
        ppe.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimators.put(id, ppe);
    }

    public static final void unregister(final String id) {
        estimators.remove(id);
    }

    public AprilTagPoseAggregator(final AprilTagFieldLayout layout, final SwerveDrivePoseEstimator aggregator) {
        setTagLayout(layout);
        this.estimator = aggregator;
    }

    public void setTagLayout(final AprilTagFieldLayout layout) {
        this.layout = layout;
        setTagLayout();
    }

    public void setTagLayout() {
        for (final PhotonPoseEstimator ppe : estimators.values()) {
            ppe.setFieldTags(layout);
        }
    }

    public void update() {
        for (final PhotonPoseEstimator ppe : estimators.values()) {
            final Optional<EstimatedRobotPose> estimatedPoseOptional = ppe.update();
            if (estimatedPoseOptional.isPresent()) {
                final EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
                estimator.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
            }
        }
    }
}
