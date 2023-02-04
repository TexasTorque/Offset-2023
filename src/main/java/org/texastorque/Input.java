/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.controllers.PathAlignController.AlignState;
import org.texastorque.controllers.PathAlignController.GridState;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input
        extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final TorqueBoolSupplier isZeroingWheels, slowModeToggle,
            alignGridLeft, alignGridCenter, alignGridRight, gridOverrideLeft,
            gridOverrideRight, gridOverrideCenter, resetGyroClick, resetPoseClick,
            toggleRotationLock, autoLevel, wantsIntakeCube, wantsIntakeCone,
            clawClose, armToHandoff, armToShelf, armToMid, armToTop, dangerMode;

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        isZeroingWheels = new TorqueBoolSupplier(() -> driver.isBButtonDown());
        slowModeToggle = new TorqueToggleSupplier(() -> driver.isLeftBumperDown());

        alignGridLeft = new TorqueBoolSupplier(() -> driver.isLeftTriggerDown());
        alignGridCenter = new TorqueBoolSupplier(() -> driver.isRightBumperDown());
        alignGridRight = new TorqueBoolSupplier(() -> driver.isRightTriggerDown());

        gridOverrideLeft = new TorqueBoolSupplier(() -> operator.isDPADLeftDown());
        gridOverrideRight = new TorqueBoolSupplier(() -> operator.isDPADRightDown());
        gridOverrideCenter = new TorqueBoolSupplier(() -> operator.isDPADDownDown());

        resetGyroClick = new TorqueClickSupplier(() -> driver.isRightCenterButtonPressed());
        resetPoseClick = new TorqueClickSupplier(() -> driver.isLeftCenterButtonPressed());
        toggleRotationLock = new TorqueToggleSupplier(() -> driver.isAButtonDown(), true);
        autoLevel = new TorqueBoolSupplier(() -> driver.isYButtonDown());

        wantsIntakeCube = new TorqueBoolSupplier(() -> operator.isLeftTriggerDown());
        wantsIntakeCone = new TorqueBoolSupplier(() -> operator.isRightTriggerDown());

        clawClose = new TorqueToggleSupplier(() -> operator.isRightBumperDown(), true);

        armToHandoff = new TorqueBoolSupplier(() -> operator.isAButtonDown());
        armToShelf = new TorqueBoolSupplier(() -> operator.isXButtonDown());
        armToMid = new TorqueBoolSupplier(() -> operator.isBButtonDown());
        armToTop = new TorqueBoolSupplier(() -> operator.isYButtonDown());
        dangerMode = new TorqueToggleSupplier(() -> operator.isLeftCenterButtonDown());
    }

    @Override
    public final void update() {
        updateDrivebase();
    }

    private void updateDrivebase() {
        updateDrivebaseSpeeds();

        driver.setRumble(driverTimeout.calculate());
        operator.setRumble(operatorTimeout.calculate());

        drivebase.setState(Drivebase.State.FIELD_RELATIVE);

        resetGyroClick.onTrue(() -> drivebase.resetGyro());
        resetPoseClick.onTrue(() -> drivebase.resetPose(Drivebase.INITIAL_POS));

        drivebase.isRotationLocked = toggleRotationLock.get();

        alignGridLeft.onTrue(() -> drivebase.setAlignState(AlignState.LEFT));
        alignGridCenter.onTrue(
                () -> drivebase.setAlignState(AlignState.CENTER));
        alignGridRight.onTrue(() -> drivebase.setAlignState(AlignState.RIGHT));

        gridOverrideLeft.onTrue(
                () -> drivebase.setGridOverride(GridState.LEFT));
        gridOverrideCenter.onTrue(
                () -> drivebase.setGridOverride(GridState.CENTER));
        gridOverrideRight.onTrue(
                () -> drivebase.setGridOverride(GridState.RIGHT));

        autoLevel.onTrue(() -> drivebase.setState(Drivebase.State.BALANCE));
        isZeroingWheels.onTrue(() -> drivebase.setState(Drivebase.State.ZERO));

        wantsIntakeCube.onTrue(
                () -> indexer.setState(Indexer.State.INTAKE_CUBE));
        wantsIntakeCone.onTrue(
                () -> indexer.setState(Indexer.State.INTAKE_CONE));

        clawClose.onTrueOrFalse(() -> hand.setState(Hand.State.CLOSE),
                () -> hand.setState(Hand.State.OPEN));

        armToHandoff.onTrue(() -> arm.setState(Arm.State.HANDOFF));
        armToShelf.onTrue(() -> arm.setState(Arm.State.SHELF));
        armToMid.onTrue(() -> arm.setState(Arm.State.MID));
        armToTop.onTrue(() -> arm.setState(Arm.State.TOP));

        lights.dangerMode = dangerMode.get();
    }

    private final static double DEADBAND = 0.125;

    private void updateDrivebaseSpeeds() {
        final double speedSetting = slowModeToggle.get() ? 0.2 : 1;

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND) *
                Drivebase.MAX_VELOCITY * speedSetting;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND) *
                Drivebase.MAX_VELOCITY * speedSetting;
        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND) *
                Drivebase.MAX_ANGULAR_VELOCITY * speedSetting;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);

        // drivebase.requestedRotation = Math.PI +
        // Math.atan2(driver.getRightXAxis(), driver.getRightYAxis()); if
        // (drivebase.requestedRotation == Math.PI)
        //     drivebase.requestedRotation = 0;
    }

    private final TorqueRequestableTimeout driverTimeout = new TorqueRequestableTimeout();

    public void setDriverRumbleFor(final double duration) {
        driverTimeout.set(duration);
    }

    private final TorqueRequestableTimeout operatorTimeout = new TorqueRequestableTimeout();

    public void setOperatorRumbleFor(final double duration) {
        operatorTimeout.set(duration);
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}