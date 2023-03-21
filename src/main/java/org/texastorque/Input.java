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
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Spindexer;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
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
            armToShelf, armToMid, armToTop, armDoHandoff, wantsOuttake,
            autoSpindex, primeRoll;

    private final TorqueRequestableTimeout driverTimeout = new TorqueRequestableTimeout();

    private final TorqueRequestableTimeout operatorTimeout = new TorqueRequestableTimeout();

    private final TorqueRequestableTimeout clawTimeout = new TorqueRequestableTimeout();

    private final TorqueTraversableSelection<Arm.State> handoffStates = new TorqueTraversableSelection<Arm.State>(
            Arm.State.STOWED, Arm.State.INDEX, Arm.State.GRAB, Arm.State.GRABBED);

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        slowModeToggle = new TorqueToggleSupplier(driver::isLeftBumperDown);

        resetGyroClick = new TorqueClickSupplier(driver::isRightCenterButtonPressed);
        resetPoseClick = new TorqueClickSupplier(driver::isLeftCenterButtonPressed);
        toggleRotationLock = new TorqueToggleSupplier(driver::isAButtonDown, true);

        wantsIntake = new TorqueBoolSupplier(operator::isRightTriggerDown);
        openClaw = new TorqueBoolSupplier(operator::isRightBumperDown);
        gamePieceModeToggle = new TorqueToggleSupplier(operator::isLeftBumperDown);

        wantsOuttake = new TorqueBoolSupplier(operator::isLeftCenterButtonDown);

        armDoHandoff = new TorqueClickSupplier(operator::isLeftTriggerDown);
        armToShelf = new TorqueClickSupplier(operator::isXButtonDown);
        armToMid = new TorqueClickSupplier(operator::isBButtonDown);
        armToTop = new TorqueClickSupplier(operator::isYButtonDown);
        armToBottom = new TorqueClickSupplier(operator::isAButtonDown);

        primeRoll = new TorqueBoolSupplier(operator::isRightCenterButtonDown);

        autoSpindex = new TorqueBoolSupplier(operator::isDPADUpDown);
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

        armToBottom.onTrue(() -> {
            arm.setState(Arm.State.STOWED);
            handoffStates.set(0);
        });

        armDoHandoff.onTrue(() -> arm.setState(handoffStates.calculate(false, true)));

        if (handoffStates.isLast())
            handoffStates.set(0);

        wantsIntake.onTrueOrFalse(() -> {
            if (!clawTimeout.get() && arm.isStowed()) {
                handoffStates.set(1);
                arm.setState(Arm.State.INDEX);
            }

            intake.setState(Intake.State.INTAKE);
        }, () -> {
            clawTimeout.set(.2);
            intake.setState(Intake.State.UP);
        });

        wantsOuttake.onTrue(() -> intake.setState(Intake.State.OUTAKE));

        primeRoll.onTrue(() -> intake.setState(Intake.State.PRIME_ROLL));

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

        spindexer.setAutoSpindex(autoSpindex.get());
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
            drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, 0);
            forks.setDirection(TorqueMath.scaledLinearDeadband(-driver.getRightYAxis(), DEADBAND));
        }
    }
}