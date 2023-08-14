package org.texastorque.toast.lib.apriltags.spoofer;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;

public final class AprilTagResultSpoofer extends PhotonPipelineResult {
    // public final List<PhotonTrackedTarget> targets;

    public AprilTagResultSpoofer() {
        this(new ArrayList<PhotonTrackedTarget>(), 0, 0);
    }

    public AprilTagResultSpoofer(final List<PhotonTrackedTarget> targets, final double latencyMillis, final double timestampSeconds) {
        super.targets.addAll(targets);
        setTimestampSeconds(timestampSeconds);
    }

    public void addTarget(final PhotonTrackedTarget target) {
        targets.add(target);
    }

    public void addTarget(final int id, final Transform3d pose, final double ambiguity,
            final List<TargetCorner> corners) {
        targets.add(new PhotonTrackedTarget(0, 0, 0, 0, id, pose, pose, ambiguity, corners, corners));
    }
    
    public PhotonTrackedTarget getBestTarget() {
        return hasTargets() ? targets.get(0) : null;
    }

    public boolean hasTargets() {
        return targets.size() > 0;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return new ArrayList<>(targets);
    } 

    public String toString() {
        return String.format("AprilTagResultSpoofer{timestamp=%f, targets=%s}", getTimestampSeconds(), targets.toString());
    }
}
