package org.texastorque.controllers;

import java.util.function.DoubleSupplier;

import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

import edu.wpi.first.math.controller.ProfiledPIDController;
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

        public double getAngle() {
            return angleSupplier.getAsDouble();
        }        
    }

    private BalanceDirection balanceDirection = BalanceDirection.POS_X;

    private static final TrapezoidProfile.Constraints BALANCE_CONSTRAINTS = new TrapezoidProfile.Constraints(0.25, 0.25);
    private final ProfiledPIDController balancePID = new ProfiledPIDController(0.05, 0.00, 0.01, BALANCE_CONSTRAINTS);

    public AutoLevelController() {
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

        SmartDashboard.putNumber("Balance Angle", gyroMeasurement);

        final boolean balanced = -3 <= gyroMeasurement && gyroMeasurement <= 3;

        final double output = balanced ? 0 : balancePID.calculate(gyroMeasurement, 0);

        SmartDashboard.putNumber("PID Out", output);

        return new ChassisSpeeds(output, 0, 0);
    }

    public void resetIf(final boolean condition) {
    }
}
