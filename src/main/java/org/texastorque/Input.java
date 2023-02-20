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
import org.texastorque.torquelib.base.TorqueDirection;
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

    public static final synchronized Input getInstance() { return instance == null ? instance = new Input() : instance; }

    private final TorqueBoolSupplier isZeroingWheels, slowModeToggle, alignGridLeft, alignGridCenter, alignGridRight, gridOverrideLeft, gridOverrideRight,
            gridOverrideCenter, resetGyroClick, resetPoseClick, toggleRotationLock, autoLevel, wantsIntake, gamePieceModeToggle, openClaw, armToBottom,
            armToShelf, armToMid, armToTop, forksUp, forksDown, spindexerRight, spindexerLeft, armDoHandoff, armToLow;

    private final TorqueRequestableTimeout driverTimeout = new TorqueRequestableTimeout();

    private final TorqueRequestableTimeout operatorTimeout = new TorqueRequestableTimeout();

    private Arm.State lastSetArmState = arm.getState();

    private final TorqueRequestableTimeout clawTimeout = new TorqueRequestableTimeout();

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        isZeroingWheels = new TorqueBoolSupplier(driver::isBButtonDown);
        slowModeToggle = new TorqueToggleSupplier(driver::isLeftBumperDown);

        alignGridLeft = new TorqueBoolSupplier(driver::isLeftTriggerDown);
        alignGridCenter = new TorqueBoolSupplier(driver::isRightBumperDown);
        alignGridRight = new TorqueBoolSupplier(driver::isRightTriggerDown);

        gridOverrideLeft = new TorqueBoolSupplier(operator::isDPADLeftDown);
        gridOverrideRight = new TorqueBoolSupplier(operator::isDPADRightDown);
        gridOverrideCenter = new TorqueBoolSupplier(operator::isDPADDownDown);

        resetGyroClick = new TorqueClickSupplier(driver::isRightCenterButtonPressed);
        resetPoseClick = new TorqueClickSupplier(driver::isLeftCenterButtonPressed);
        toggleRotationLock = new TorqueToggleSupplier(driver::isAButtonDown, true);
        autoLevel = new TorqueBoolSupplier(driver::isYButtonDown);

        wantsIntake = new TorqueBoolSupplier(operator::isRightTriggerDown);
        openClaw = new TorqueBoolSupplier(operator::isRightBumperDown);
        gamePieceModeToggle = new TorqueToggleSupplier(operator::isLeftBumperDown);

        armDoHandoff = new TorqueBoolSupplier(operator::isLeftTriggerDown);
        armToShelf = new TorqueBoolSupplier(operator::isXButtonDown);
        armToMid = new TorqueBoolSupplier(operator::isBButtonDown);
        armToTop = new TorqueBoolSupplier(operator::isYButtonDown);
        armToLow = new TorqueBoolSupplier(operator::isLeftCenterButtonDown);
        armToBottom = new TorqueBoolSupplier(operator::isAButtonDown);

        forksUp = new TorqueBoolSupplier(driver::isDPADUpDown);
        forksDown = new TorqueBoolSupplier(driver::isDPADDownDown);

        spindexerRight = new TorqueBoolSupplier(operator::isRightBumperDown);
        spindexerLeft = new TorqueBoolSupplier(operator::isLeftBumperDown);
    }

    public void setDriverRumbleFor(final double duration) { driverTimeout.set(duration); }

    public void setOperatorRumbleFor(final double duration) { operatorTimeout.set(duration); }

    public void update() {
        updateDrivebaseSpeeds();

        driver.setRumble(driverTimeout.get());
        operator.setRumble(operatorTimeout.get());

        drivebase.setState(Drivebase.State.FIELD_RELATIVE);

        resetGyroClick.onTrue(() -> drivebase.resetGyro());
        resetPoseClick.onTrue(() -> drivebase.resetPose(Drivebase.INITIAL_POS));

        drivebase.isRotationLocked = toggleRotationLock.get();

        // alignGridLeft.onTrue(() -> drivebase.setAlignState(AlignState.LEFT));
        // alignGridCenter.onTrue(() -> drivebase.setAlignState(AlignState.CENTER));
        // alignGridRight.onTrue(() -> drivebase.setAlignState(AlignState.RIGHT));

        // gridOverrideLeft.onTrue(() -> drivebase.setGridOverride(GridState.LEFT));
        // gridOverrideCenter.onTrue(() -> drivebase.setGridOverride(GridState.CENTER));
        // gridOverrideRight.onTrue(() -> drivebase.setGridOverride(GridState.RIGHT));

        // autoLevel.onTrue(() -> drivebase.setState(Drivebase.State.BALANCE));
        isZeroingWheels.onTrue(() -> drivebase.setState(Drivebase.State.ZERO));

        openClaw.onTrue(() -> hand.setState(Hand.State.OPEN));

        gamePieceModeToggle.onTrueOrFalse(() -> hand.setGamePieceMode(GamePiece.CONE), () -> hand.setGamePieceMode(GamePiece.CUBE));

        armToShelf.onTrue(() -> arm.setState(lastSetArmState = Arm.State.SHELF));
        armToMid.onTrue(() -> arm.setState(lastSetArmState = Arm.State.MID));
        armToTop.onTrue(() -> arm.setState(lastSetArmState = Arm.State.TOP));
        armToBottom.onTrue(() -> arm.setState(lastSetArmState = Arm.State.BOTTOM));

        armDoHandoff.onTrueOrFalse(() -> {
            arm.setState(Arm.State.GRAB);
        }, () -> {
            if (arm.isState(Arm.State.GRAB))
                arm.setState(Arm.State.BOTTOM);
        });

        wantsIntake.onTrueOrFalse(() -> {
            if (!clawTimeout.get())
                arm.setState(Arm.State.HANDOFF);

            intake.setState(Intake.State.INTAKE);
        }, () -> {
            clawTimeout.set(.5);
            intake.setState(Intake.State.UP);
        });

        forksUp.onTrue(() -> forks.setDirection(TorqueDirection.FORWARD));
        forksDown.onTrue(() -> forks.setDirection(TorqueDirection.REVERSE));

        updateSpindexer();
        
    }

    private void updateSpindexer() {
        final double input = operator.getLeftXAxis();
        if (Math.abs(input) < .1)
            spindexer.setDirection(TorqueDirection.NEUTRAL);
        else if (input > 0)
            spindexer.setDirection(TorqueDirection.FORWARD);
        else 
            spindexer.setDirection(TorqueDirection.REVERSE);
    }

    private void updateDrivebaseSpeeds() {
        drivebase.speedSetting = slowModeToggle.get() ? SpeedSetting.SLOW : SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND) * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND) * Drivebase.MAX_VELOCITY;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND) * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);

        // drivebase.requestedRotation = Math.PI +
        // Math.atan2(driver.getRightXAxis(), driver.getRightYAxis()); if
        // (drivebase.requestedRotation == Math.PI)
        //     drivebase.requestedRotation = 0;
    }
}