package org.texastorque.toast.lib.pipelines;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.texastorque.toast.lib.apriltags.AprilTagPoseAggregator;
import org.texastorque.toast.lib.apriltags.spoofer.AprilTagCameraSpoofer;
import org.texastorque.toast.lib.apriltags.spoofer.AprilTagResultSpoofer;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class AprilTagPipeline extends Pipeline {

    private final AprilTagCameraSpoofer spoofer;
    private final Transform3d robotToCamera;

    public AprilTagPipeline(final Transform3d robotToCamera) {
        super("april-tags");
        spoofer = new AprilTagCameraSpoofer();
        this.robotToCamera = robotToCamera;
    }

    @Override
    protected void init() {
        AprilTagPoseAggregator.register(name, spoofer, robotToCamera);
    }

    @Override
    protected void deinit() {
        AprilTagPoseAggregator.unregister(name);
    }

    private static AprilTagResultSpoofer parseAprilTagResultFromJSON(final JsonNode root) {
        final AprilTagResultSpoofer result = new AprilTagResultSpoofer();

            result.setTimestampSeconds(root.get("timestamp").asDouble(-1));

            root.get("targets").forEach(target -> {
                final int id = target.get("id").asInt(-1);
                
                final JsonNode translation = target.get("translation");
                final JsonNode rotation = target.get("quat_rot");

                final Transform3d transform = new Transform3d(
                    new Translation3d(
                        translation.get(0).asDouble(),
                        translation.get(1).asDouble(),
                        translation.get(2).asDouble()
                    ), 
                    new Rotation3d(
                        new Quaternion(
                            rotation.get(0).asDouble(),
                            rotation.get(1).asDouble(),
                            rotation.get(2).asDouble(),
                            rotation.get(3).asDouble()
                        )
                    )
                );

                // Currently does not exist! -- will always return 0
                // final double ambguity = target.get("ambiguity").asDouble(0);
                final double ambguity = .1;

                final List<TargetCorner> corners = new ArrayList<TargetCorner>();
                target.get("corners").forEach(corner -> {
                    corners.add(new TargetCorner(corner.get(0).asDouble(), corner.get(1).asDouble()));
                });

                result.addTarget(id, transform, ambguity, corners);
            });

        return result;
    }

    @Override
    protected void update(final JsonNode json) {
        spoofer.setCameraMatrix(getIntrinsics());
        spoofer.setDistCoeffs(getDistorion());
        final AprilTagResultSpoofer result = parseAprilTagResultFromJSON(json);
        spoofer.setResult(result);
    }

    public static void main(final String[] args){ 
        final String json = "{'targets': [{'id': 0, 'hamming': 0, 'decision_margin': 63.43700408935547, 'center': [203.14721608633693, 222.35142155829502], 'corners': [[280.58865356445324, 149.44662475585935], [123.44876098632812, 139.48097229003906], [117.42034912109368, 303.0562744140625], [277.7310485839844, 299.9036865234375]], 'euler_rot': [0.019671437795993008, 0.2616517931891449, -3.124919318485002], 'quat_rot': [-0.13052347701498948, 0.00866369327450734, 0.991382779496398, -0.0069818969312930605], 'translation': [-0.125655091097458, -0.0341491545178374, 0.5015440915987419], 'pose_error': 1.1621206675884188e-05, 'homography': [[-87.74010035320873, 3.9491869872144605, 203.14721608633693], [-11.089129932267383, -76.4052625285758, 222.35142155829502], [-0.04196474641512287, 0.008813678447428079, 1.0]]}], 'timestamp': 1690059119.9622421}"
                .replace("'", "\"");
        parseAndHandleJSON(json, root -> {
            final var result = parseAprilTagResultFromJSON(root);
            System.out.println(result.toString());
        });
    }
}

/*
timestamp: 100,
targets: [
    {
        id: 0 (int),
        transform: {
            x: 0, y: 0, z: 0,
            qw: 
        }
    }
]

 */
