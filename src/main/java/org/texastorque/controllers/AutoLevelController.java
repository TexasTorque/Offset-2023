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
        POS_X(0, 0, () -> -TorqueNavXGyro.getInstance().getRoll()), 
        NEG_X(135, 225, () -> TorqueNavXGyro.getInstance().getRoll()), 
        POS_Y(45, 135, () -> TorqueNavXGyro.getInstance().getPitch()), 
        NEG_Y(225, 315, () -> -TorqueNavXGyro.getInstance().getPitch());

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

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(.5, .5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(.25, .25);
    public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1 * Math.PI, 1 * Math.PI);

    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4 * Math.PI, 0, 0, OMEGA_CONSTRAINTS);


    private final PIDController rotationPID = new PIDController(.05, 0, 0);

    private final Supplier<Pose2d> poseSupplier;

    public AutoLevelController(final Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    public ChassisSpeeds calculate() {

        final Rotation2d startingRotation = TorqueNavXGyro.getInstance().getHeadingCCW();

        for (final BalanceDirection direction : BalanceDirection.values()) {
            if (direction.isConstrained(startingRotation)) {
                balanceDirection = direction;
                break;
            }
        }

        final double gyroMeasurement = balanceDirection.getAngle();

        final double xOffset = -rotationPID.calculate(gyroMeasurement);

        SmartDashboard.putNumber("x offset", xOffset);

        final Pose2d robotPose = poseSupplier.get();

        final Pose2d goalPose = new Pose2d(robotPose.getX() + xOffset, staticYPos, balanceDirection.getCenterHeading());

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().plus(new Rotation2d(Math.PI)).getRadians());

        final boolean xAtGoal = xController.atGoal();
        final boolean yAtGoal = yController.atGoal();
        final boolean omegaAtGoal = omegaController.atGoal();

        final double xSpeed = xAtGoal ? 0 : -xController.calculate(robotPose.getX());
        final double ySpeed = yAtGoal ? 0 : -yController.calculate(robotPose.getY());
        final double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        return new ChassisSpeeds(xSpeed, ySpeed, 0);
    }

    private double staticYPos = 0;

    public void resetIf(final boolean notInLoop) {
        if (!notInLoop) return;

        staticYPos = poseSupplier.get().getY();

        final Pose2d robotPose = poseSupplier.get();
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        omegaController.reset(robotPose.getRotation().getRadians());
    }
}
