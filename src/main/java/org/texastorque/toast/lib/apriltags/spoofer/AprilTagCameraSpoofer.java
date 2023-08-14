package org.texastorque.toast.lib.apriltags.spoofer;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.texastorque.toast.lib.matrix.DistortionMatrix;
import org.texastorque.toast.lib.matrix.IntrinsicsMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTagCameraSpoofer extends PhotonCamera {
    public AprilTagCameraSpoofer() {
        super("camera-name");
    }

    private AprilTagResultSpoofer result = new AprilTagResultSpoofer();
    private IntrinsicsMatrix cameraMatrix = null;
    private DistortionMatrix distCoeffs = null;

    public void setResult(final AprilTagResultSpoofer result) {
        this.result = result;
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        return result;
    }

    @Override
    public Optional<Matrix<N3, N3>> getCameraMatrix() {
        if (cameraMatrix != null)
            return Optional.of(cameraMatrix);
        return Optional.empty();
    }

    public void setCameraMatrix(final IntrinsicsMatrix cameraMatrix) {
        this.cameraMatrix = cameraMatrix;
    }

    @Override
    public Optional<Matrix<N5, N1>> getDistCoeffs() {
        if (distCoeffs != null)
            return Optional.of(distCoeffs);
        return Optional.empty();
    }

    public void setDistCoeffs(final DistortionMatrix distCoeffs) {
        this.distCoeffs = distCoeffs;
    }
}
