package org.texastorque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.controllers.PathAlignController.AlignState;
import org.texastorque.controllers.PathAlignController.GridState;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

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
    }

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
            autoLevel;

    @Override
    public final void update() {
        updateDrivebase();
    }

    private void updateDrivebase() {
        updateDrivebaseSpeeds();
        drivebase.state = Drivebase.State.FIELD_RELATIVE;

        resetGyroClick.onTrue(() -> drivebase.resetGyro());
        resetPoseClick.onTrue(() -> drivebase.resetPose(Drivebase.INITIAL_POS));
    
        toggleRotationLockClick.onTrue(() -> drivebase.isRotationLocked = !drivebase.isRotationLocked);

        alignGridLeft.onTrue(() -> drivebase.setAlignState(AlignState.LEFT));
        alignGridCenter.onTrue(() -> drivebase.setAlignState(AlignState.CENTER));
        alignGridRight.onTrue(() -> drivebase.setAlignState(AlignState.RIGHT));

        gridOverrideLeft.onTrue(() -> drivebase.setGridOverride(GridState.LEFT));
        gridOverrideCenter.onTrue(() -> drivebase.setGridOverride(GridState.CENTER));
        gridOverrideRight.onTrue(() -> drivebase.setGridOverride(GridState.RIGHT));

        autoLevel.onTrue(() -> drivebase.state = Drivebase.State.BALANCE);
        isZeroingWheels.onTrue(() -> drivebase.state = Drivebase.State.ZERO);
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

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}