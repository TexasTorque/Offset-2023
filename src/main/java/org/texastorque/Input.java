/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.controllers.PathAlignController.AlignState;
import org.texastorque.controllers.PathAlignController.GridState;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Drivebase.SpeedSetting;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Spindexer;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }

    private final TorqueBoolSupplier isZeroingWheels, alignGridLeft, alignGridCenter, alignGridRight,
            gridOverrideLeft, gridOverrideRight,
            gridOverrideCenter, resetGyroClick, resetPoseClick, wantsIntake,
            gamePieceModeToggle, openClaw,
            armToShelf, armToMid, armToTop, armDoHandoff, armThrow,
            wantsOuttake, xFactorToggle, autoSpindex, wantsTipCone, slowMode, armLeavingHandoff, wantsSlowIntake,
            wantsSlowOuttake, armToPrime, armLeavingShelf;

    private final TorqueRequestableTimeout driverTimeout = new TorqueRequestableTimeout();

    private final TorqueRequestableTimeout operatorTimeout = new TorqueRequestableTimeout();

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        isZeroingWheels = new TorqueBoolSupplier(driver::isBButtonDown);
        xFactorToggle = new TorqueToggleSupplier(driver::isXButtonDown);

        alignGridLeft = new TorqueBoolSupplier(() -> false);
        alignGridCenter = new TorqueBoolSupplier(() -> false);
        alignGridRight = new TorqueBoolSupplier(() -> false);

        gridOverrideLeft = new TorqueBoolSupplier(operator::isDPADLeftDown);
        gridOverrideRight = new TorqueBoolSupplier(operator::isDPADRightDown);
        gridOverrideCenter = new TorqueBoolSupplier(operator::isDPADDownDown);

        resetGyroClick = new TorqueClickSupplier(driver::isRightCenterButtonPressed);
        resetPoseClick = new TorqueClickSupplier(driver::isLeftCenterButtonPressed);

        wantsIntake = new TorqueBoolSupplier(() -> operator.isRightTriggerDown() || driver.isRightTriggerDown());

        openClaw = new TorqueBoolSupplier(operator::isRightBumperDown);
        gamePieceModeToggle = new TorqueToggleSupplier(() -> operator.isLeftBumperDown() || driver.isYButtonDown());
        wantsOuttake = new TorqueBoolSupplier(() -> operator.isLeftCenterButtonDown() || driver.isLeftTriggerDown());

        wantsTipCone = new TorqueBoolSupplier(operator::isLeftStickClickDown);

        wantsSlowIntake = new TorqueBoolSupplier(driver::isRightBumperDown);
        wantsSlowOuttake = new TorqueBoolSupplier(driver::isLeftBumperDown);

        armDoHandoff = new TorqueBoolSupplier(operator::isLeftTriggerDown);
        armLeavingHandoff = new TorqueClickSupplier(() -> !armDoHandoff.get());
        armToShelf = new TorqueBoolSupplier(operator::isXButtonDown);
        armLeavingShelf = new TorqueClickSupplier(() -> !armToShelf.get());

        armToMid = new TorqueClickSupplier(operator::isBButtonDown);
        armToTop = new TorqueClickSupplier(operator::isYButtonDown);

        armThrow = new TorqueClickSupplier(operator::isRightCenterButtonDown);

        autoSpindex = new TorqueBoolSupplier(operator::isDPADUpDown);

        slowMode = new TorqueToggleSupplier(driver::isAButtonDown);

        armToPrime = new TorqueClickSupplier(operator::isAButtonDown);

    }

    public void setDriverRumbleFor(final double duration) {
        driverTimeout.set(duration);
    }

    public void setOperatorRumbleFor(final double duration) {
        operatorTimeout.set(duration);
    }

    public void update() {
        updateDrivebaseSpeeds();

        driver.setRumble(driverTimeout.get());
        operator.setRumble(operatorTimeout.get());

        drivebase.setState(Drivebase.State.FIELD_RELATIVE);

        resetGyroClick.onTrue(() -> drivebase.resetGyro());
        resetPoseClick.onTrue(() -> drivebase.resetPose(Drivebase.INITIAL_POS));

        drivebase.isRotationLocked = true;

        alignGridLeft.onTrue(() -> drivebase.setAlignState(AlignState.LEFT));
        alignGridCenter.onTrue(() -> drivebase.setAlignState(AlignState.CENTER));
        alignGridRight.onTrue(() -> drivebase.setAlignState(AlignState.RIGHT));

        gridOverrideLeft.onTrue(() -> drivebase.setGridOverride(GridState.LEFT));
        gridOverrideCenter.onTrue(() -> drivebase.setGridOverride(GridState.CENTER));
        gridOverrideRight.onTrue(() -> drivebase.setGridOverride(GridState.RIGHT));

        xFactorToggle.onTrue(() -> drivebase.setState(Drivebase.State.XF));
        isZeroingWheels.onTrue(() -> drivebase.setState(Drivebase.State.ZERO));

        openClaw.onTrue(() -> hand.setState(Hand.State.OPEN));

        gamePieceModeToggle.onTrueOrFalse(() -> hand.setGamePieceMode(GamePiece.CONE),
                () -> hand.setGamePieceMode(GamePiece.CUBE));

        armToShelf.onTrue(() -> arm.setState(Arm.State.SHELF));
        armToMid.onTrue(() -> arm.setState(Arm.State.MID));
        armToTop.onTrue(() -> arm.setState(Arm.State.TOP));

        armThrow.onTrue(() -> arm.setState(Arm.State.THROW));

        armDoHandoff.onTrue(() -> arm.setState(Arm.State.HANDOFF));
        armLeavingHandoff.onTrue(() -> arm.setState(Arm.State.PRIME));

        armToShelf.onTrue(() -> arm.setState(Arm.State.SHELF));
        armLeavingShelf.onTrue(() -> arm.setState(Arm.State.PRIME));

        wantsIntake.onTrueOrFalse(() -> {
            intake.setState(Intake.State.INTAKE);
        }, () -> {
            intake.setState(Intake.State.UP);
        });

        wantsOuttake.onTrue(() -> intake.setState(Intake.State.OUTAKE));

        wantsTipCone.onTrue(() -> {
            intake.setState(Intake.State.UP_ROLL);
            spindexer.setState(Spindexer.State.FAST_CW);
        });

        wantsSlowIntake.onTrue(() -> intake.setState(Intake.State.SLOW_INTAKE));
        wantsSlowOuttake.onTrue(() -> intake.setState(Intake.State.SLOW_OUTAKE));

        arm.setSetpointAdjustment(operator.getRightYAxis());

        armToPrime.onTrue(() -> arm.setState(Arm.State.PRIME));

        updateSpindexer();
    }

    private void updateSpindexer() {
        final double fast = operator.getLeftYAxis();
        if (Math.abs(fast) > DEADBAND) {
            if (fast < 0)
                spindexer.setState(Spindexer.State.FAST_CCW);
            else
                spindexer.setState(Spindexer.State.FAST_CW);
        }

        final double slow = operator.getLeftXAxis();
        if (Math.abs(slow) > DEADBAND) {
            if (slow < 0)
                spindexer.setState(Spindexer.State.SLOW_CCW);
            else
                spindexer.setState(Spindexer.State.SLOW_CW);
        }

        autoSpindex.onTrue(() -> spindexer.setState(Spindexer.State.AUTO_SPINDEX));
    }

    private void updateDrivebaseSpeeds() {
        SmartDashboard.putBoolean("slowMode", slowMode.get());
        drivebase.speedSetting = slowMode.get() ? SpeedSetting.SLOW : SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;

        if (!driver.isRightStickClickDown()) {
            final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                    * Drivebase.MAX_ANGULAR_VELOCITY;
            drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
        } else {
            drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, 0);
            forks.setDirection(TorqueMath.scaledLinearDeadband(-driver.getRightYAxis(), DEADBAND));
        }
    }
}