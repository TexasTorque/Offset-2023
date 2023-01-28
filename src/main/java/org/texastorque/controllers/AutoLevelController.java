package org.texastorque.controllers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class AutoLevelController {
    private static enum BalanceDirection {
        POS_X(0, 0, () -> -TorqueNavXGyro.getInstance().getPitch()), 
        NEG_X(135, 225, () -> TorqueNavXGyro.getInstance().getPitch()), 
        POS_Y(45, 135, () -> TorqueNavXGyro.getInstance().getRoll()), 
        NEG_Y(225, 315, () -> -TorqueNavXGyro.getInstance().getRoll());

        private final double lower, upper;
        private final DoubleSupplier angleSupplier;

        private BalanceDirection(final double lower, final double upper, final DoubleSupplier angleSupplier) {
            this.lower = lower;
            this.upper = upper;
            this.angleSupplier = angleSupplier;
        }

        public boolean isConstrained(final Rotation2d angle) {
            if (lower == 0 && upper == 0)
                return 45 >= angle.getDegrees() && angle.getDegrees() >= 0
                        || 360 >= angle.getDegrees() && angle.getDegrees() >= 315;
            return lower <= angle.getDegrees() && angle.getDegrees() <= upper;
        }

        public Rotation2d getCenterHeading() {
            return Rotation2d.fromDegrees((lower + upper) / 2.0);
        }

        public double getAngle() {
            return angleSupplier.getAsDouble();
        }        
    }

    private BalanceDirection balanceDirection = BalanceDirection.POS_X;

    private final Supplier<Pose2d> poseSupplier;

    public AutoLevelController(final Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    private double lastGyroMeasure = 0;

    private final PIDController controller = new PIDController(.02, 0, .0007);

    public ChassisSpeeds calculate() {

        final Rotation2d startingRotation = TorqueNavXGyro.getInstance().getHeadingCCW();

        for (final BalanceDirection direction : BalanceDirection.values()) {
            if (direction.isConstrained(startingRotation)) {
                balanceDirection = direction;
                break;
            }
        }

        final double gyroMeasurement = -balanceDirection.getAngle();

        double drivePower = -controller.calculate(gyroMeasurement, 0);

        if (Math.abs(drivePower) > 0.4)
            drivePower = Math.copySign(.4, drivePower);

        lastGyroMeasure = gyroMeasurement;

        return new ChassisSpeeds(drivePower * 3.5, 0, 0);
    }

    private double staticYPos = 0;

    public void resetIf(final boolean notInLoop) {
        if (!notInLoop) return;

        staticYPos = poseSupplier.get().getY();

        final Pose2d robotPose = poseSupplier.get();
    }
}
