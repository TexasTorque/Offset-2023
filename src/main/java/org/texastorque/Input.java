package org.texastorque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.controllers.PathAlignController.AlignState;
import org.texastorque.controllers.PathAlignController.GridState;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
import org.texastorque.torquelib.control.TorqueTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final TorqueBoolSupplier 
            isZeroingWheels,
            slowModeToggle,
            alignGridLeft,
            alignGridCenter,
            alignGridRight,
            gridOverrideLeft,
            gridOverrideRight,
            gridOverrideCenter,
            resetGyroClick,
            resetPoseClick,
            toggleRotationLockClick,
            autoLevel,
            wantsIntakeCube,
            wantsIntakeCone,
            clawClose,
            armToHandoff,
            armToShelf,
            armToMid,
            armToTop;
            
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
        toggleRotationLockClick = new TorqueClickSupplier(() -> driver.isAButtonDown());
        autoLevel = new TorqueBoolSupplier(() -> driver.isYButtonDown());

        wantsIntakeCube = new TorqueBoolSupplier(() -> operator.isLeftTriggerDown());
        wantsIntakeCone = new TorqueBoolSupplier(() -> operator.isRightTriggerDown());

        clawClose = new TorqueToggleSupplier(() -> operator.isRightBumperDown());

        armToHandoff = new TorqueBoolSupplier(() -> operator.isAButtonDown());
        armToShelf = new TorqueBoolSupplier(() -> operator.isXButtonDown());
        armToMid = new TorqueBoolSupplier(() -> operator.isBButtonDown());
        armToTop = new TorqueBoolSupplier(() -> operator.isYButtonDown());
    }


    @Override
    public final void update() {
        updateDrivebase();
    }

    private void updateDrivebase() {
        updateDrivebaseSpeeds();
        drivebase.setState(Drivebase.State.FIELD_RELATIVE);

        resetGyroClick.onTrue(() -> drivebase.resetGyro());
        resetPoseClick.onTrue(() -> drivebase.resetPose(Drivebase.INITIAL_POS));
    
        toggleRotationLockClick.onTrue(() -> drivebase.isRotationLocked = !drivebase.isRotationLocked);

        alignGridLeft.onTrue(() -> drivebase.setAlignState(AlignState.LEFT));
        alignGridCenter.onTrue(() -> drivebase.setAlignState(AlignState.CENTER));
        alignGridRight.onTrue(() -> drivebase.setAlignState(AlignState.RIGHT));

        gridOverrideLeft.onTrue(() -> drivebase.setGridOverride(GridState.LEFT));
        gridOverrideCenter.onTrue(() -> drivebase.setGridOverride(GridState.CENTER));
        gridOverrideRight.onTrue(() -> drivebase.setGridOverride(GridState.RIGHT));

        autoLevel.onTrue(() -> drivebase.setState(Drivebase.State.BALANCE));
        isZeroingWheels.onTrue(() -> drivebase.setState(Drivebase.State.ZERO));

        wantsIntakeCube.onTrue(() -> indexer.state = Indexer.State.INTAKE_CUBE);
        wantsIntakeCone.onTrue(() -> indexer.state = Indexer.State.INTAKE_CONE);

        clawClose.onTrueOrFalse(() -> hand.state = Hand.State.CLOSE, () -> hand.state = Hand.State.OPEN);

        armToHandoff.onTrue(() -> arm.state = Arm.State.HANDOFF);
        armToShelf.onTrue(() -> arm.state = Arm.State.SHELF);
        armToMid.onTrue(() -> arm.state = Arm.State.MID);
        armToTop.onTrue(() -> arm.state = Arm.State.TOP);
    }

    private final static double DEADBAND = 0.125;

    private void updateDrivebaseSpeeds() {
        final double speedSetting = slowModeToggle.get() ? 0.2 : 1;
        
        final double xVelocity = TorqueMath
                .scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND) * Drivebase.MAX_VELOCITY * speedSetting;
        final double yVelocity = TorqueMath
                .scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND) * Drivebase.MAX_VELOCITY * speedSetting;
        final double rotationVelocity = TorqueMath
                .scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND) * Drivebase.MAX_ANGULAR_VELOCITY * speedSetting;

        drivebase.inputSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);

        // drivebase.requestedRotation = Math.PI + Math.atan2(driver.getRightXAxis(), driver.getRightYAxis());
        // if (drivebase.requestedRotation == Math.PI)
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