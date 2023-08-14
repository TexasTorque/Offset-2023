package org.texastorque.toast.lib;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.toast.lib.apriltags.AprilTagPoseAggregator;
import org.texastorque.toast.lib.pipelines.Pipeline;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public final class Toast {
    public final Map<String, Pipeline> pipelines = new HashMap<>();
    private final AprilTagPoseAggregator aggregator;

    public Toast(final AprilTagFieldLayout layout, final SwerveDrivePoseEstimator spe) {
        aggregator = new AprilTagPoseAggregator(layout, spe);
    }

    public void setPipeline(final String name, final Pipeline pipeline) {
        if (pipelines.containsKey(name)) {
            pipelines.get(name).runDeinit();
        }
        pipeline.setName(name);
        pipeline.runInit();
        pipelines.put(name, pipeline);
        aggregator.setTagLayout();
    }

    public Pipeline getPipeline(final String name) {
        return pipelines.get(name);
    }

    public void update() {
        for (final Pipeline pipeline : pipelines.values()) {
            pipeline.runUpdate();
        }
        aggregator.update();
    }

    public void update(final Rotation2d gyroAngle, final SwerveModulePosition[] modulePositions) {
        update();
        aggregator.estimator.update(gyroAngle, modulePositions);
    }

    public SwerveDrivePoseEstimator getEstimator() {
        return aggregator.estimator;
    }
}
