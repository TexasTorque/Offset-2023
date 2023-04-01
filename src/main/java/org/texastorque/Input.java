/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Drivebase.SpeedSetting;
import org.texastorque.subsystems.Forks;
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

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }

    private final TorqueBoolSupplier slowModeToggle, resetGyroClick, resetPoseClick, toggleRotationLock, wantsIntake,
            gamePieceModeToggle, openClaw, armToBottom,
            armToShelf, armToMid, armToTop, armToLow, armDoHandoff, wantsOuttake,
            autoSpindex, primeRoll, forksDownAuto, forksDown, forksUpAuto, forksUp;

    private final TorqueRequestableTimeout driverTimeout = new TorqueRequestableTimeout();

    private final TorqueRequestableTimeout operatorTimeout = new TorqueRequestableTimeout();

    private final TorqueRequestableTimeout clawTimeout = new TorqueRequestableTimeout();

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        slowModeToggle = new TorqueToggleSupplier(driver::isLeftBumperDown);

        resetGyroClick = new TorqueClickSupplier(driver::isRightCenterButtonPressed);
        resetPoseClick = new TorqueClickSupplier(driver::isLeftCenterButtonPressed);
        toggleRotationLock = new TorqueToggleSupplier(driver::isAButtonDown, true);

        wantsIntake = new TorqueBoolSupplier(operator::isRightTriggerDown);
        openClaw = new TorqueBoolSupplier(operator::isRightBumperDown);
        gamePieceModeToggle = new TorqueToggleSupplier(() -> operator.isLeftBumperDown() || driver.isYButtonDown());

        wantsOuttake = new TorqueBoolSupplier(operator::isLeftCenterButtonDown);

        armDoHandoff = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        armToShelf = new TorqueClickSupplier(operator::isXButtonDown);
        armToMid = new TorqueClickSupplier(operator::isBButtonDown);
        armToTop = new TorqueClickSupplier(operator::isYButtonDown);
        // armToTop = new TorqueBoolSupplier(() -> operator::isYButtonDown ||
        // driver::isYButtonDown);
        // armToTop = new TorqueBoolSupplier(() -> operator.isYButtonDown() ||
        // driver.isYButtonDown());

        armToBottom = new TorqueClickSupplier(operator::isAButtonDown);
        armToLow = new TorqueClickSupplier(operator::isLeftBumperDown);

        primeRoll = new TorqueBoolSupplier(operator::isRightCenterButtonDown);

        autoSpindex = new TorqueBoolSupplier(operator::isDPADUpDown);

        forksDownAuto = new TorqueBoolSupplier(driver::isDPADDownDown);
        forksDown = new TorqueBoolSupplier(driver::isDPADLeftDown);
        forksUpAuto = new TorqueBoolSupplier(driver::isDPADRightDown);
        forksUp = new TorqueBoolSupplier(driver::isDPADUpDown);
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

        drivebase.isRotationLocked = toggleRotationLock.get();

        openClaw.onTrue(() -> hand.setState(Hand.State.OPEN));

        gamePieceModeToggle.onTrueOrFalse(() -> hand.setGamePieceMode(GamePiece.CONE),
                () -> hand.setGamePieceMode(GamePiece.CUBE));

        armToShelf.onTrue(() -> arm.setState(Arm.State.SHELF));
        armToMid.onTrue(() -> arm.setState(Arm.State.MID));
        armToTop.onTrue(() -> arm.setState(Arm.State.TOP));
        // armToLow.onTrue(()->arm.setState(Arm.State.LOW));
        armToBottom.onTrue(() -> arm.setState(Arm.State.STOWED));

        armDoHandoff.onTrue(() -> arm.setState(Arm.State.HANDOFF));

        armToShelf.onTrue(() -> arm.setState(Arm.State.SHELF));

        wantsIntake.onTrueOrFalse(() -> {

            intake.setState(Intake.State.INTAKE);
        }, () -> {
            intake.setState(Intake.State.UP);
        });

        wantsOuttake.onTrue(() -> intake.setState(Intake.State.OUTAKE));

        primeRoll.onTrue(() -> intake.setState(Intake.State.PRIME_ROLL));

        updateSpindexer();
    }

    private void updateSpindexer() {
        final double fast = operator.getLeftXAxis();
        if (Math.abs(fast) > DEADBAND) {
            if (fast < 0)
                spindexer.setState(Spindexer.State.FAST_CCW);
            else
                spindexer.setState(Spindexer.State.FAST_CW);
        }

        // final double slow = operator.getLeftXAxis();
        // if (Math.abs(slow) > DEADBAND) {
        // if (slow < 0)
        // spindexer.setState(Spindexer.State.SLOW_CCW);
        // else
        // spindexer.setState(Spindexer.State.SLOW_CW);
        // }

        autoSpindex.onTrue(() -> spindexer.setState(Spindexer.State.AUTO_SPINDEX));

    }

    private void updateDrivebaseSpeeds() {
        drivebase.speedSetting = SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;

        if (!driver.isRightStickClickDown()) {
            final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                    * Drivebase.MAX_ANGULAR_VELOCITY;
            drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
        } else {
            final double joystick = -driver.getRightYAxis();
            if (joystick > 0) {
                // drivebase.setState(Drivebase.State.STRAIGHT);
                forks.setState(Forks.State.UP);
                forks.speed = TorqueMath.scaledLinearDeadband(-driver.getRightYAxis(), DEADBAND);
            } else {
                forks.setState(Forks.State.DOWN_AUTO);
            }
            drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, 0);
        }
    }
}