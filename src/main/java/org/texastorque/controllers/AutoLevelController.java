/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.controllers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AutoLevelController extends AbstractController<TorqueSwerveSpeeds> {
    private static enum BalanceDirection {
        POS_X(315, 45, () -> - TorqueNavXGyro.getInstance().getPitch()),
        NEG_X(135, 225, () -> TorqueNavXGyro.getInstance().getPitch()),
        POS_Y(45, 135, () -> TorqueNavXGyro.getInstance().getRoll()),
        NEG_Y(225, 315, () -> - TorqueNavXGyro.getInstance().getRoll());

        private final double lower, upper;
        private final DoubleSupplier angleSupplier;

        private BalanceDirection(final double lower, final double upper, final DoubleSupplier angleSupplier) {
            this.lower = (lower + 45) % 360;
            this.upper = (upper + 45) % 360;
            this.angleSupplier = angleSupplier;
        }

        public boolean isConstrained(final Rotation2d angle) {
            final double theta = (angle.getDegrees() + 45) % 360;
            return lower <= theta && theta <= upper;
        }

        public double getAngle() { return angleSupplier.getAsDouble(); }
    }

    private static final double TILT_THRESHOLD_DEG = 10, FLAT_THRESHOLD_DEG = 2;

    private BalanceDirection balanceDirection = BalanceDirection.POS_X;

    private final Supplier<Pose2d> poseSupplier;

    private final PIDController controller = new PIDController(.0175, 0, .015);

    private boolean hasTilted = false, isFlat = false;

    public AutoLevelController(final Supplier<Pose2d> poseSupplier) { this.poseSupplier = poseSupplier; }

    public final boolean isDone() { return isFlat && hasTilted; }

    public TorqueSwerveSpeeds calculate() {

        final Rotation2d startingRotation = TorqueNavXGyro.getInstance().getHeadingCCW();

        for (final BalanceDirection direction : BalanceDirection.values()) {
            if (direction.isConstrained(startingRotation)) {
                balanceDirection = direction;
                break;
            }
        }

        final double gyroMeasurement = -balanceDirection.getAngle();

        isFlat = Math.abs(gyroMeasurement) <= FLAT_THRESHOLD_DEG;

        if (Math.abs(gyroMeasurement) >= TILT_THRESHOLD_DEG) hasTilted = true;

        double drivePower = -controller.calculate(gyroMeasurement, 0);

        if (Math.abs(drivePower) > 0.4) drivePower = Math.copySign(.4, drivePower);

        return isFlat ? new TorqueSwerveSpeeds() : new TorqueSwerveSpeeds(drivePower * 3.5, 0, 0);
    }
    public void resetIf(final boolean notInLoop) {
        if (!notInLoop) return;
        hasTilted = false;
    }
}
