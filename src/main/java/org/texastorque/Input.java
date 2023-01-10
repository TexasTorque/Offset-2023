package org.texastorque;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueSlewLimiter;
import org.texastorque.torquelib.control.TorqueTraversableSelection;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.util.TorqueMath;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private Input() {
        driver = new TorqueController(0, .2);
        operator = new TorqueController(1);
    }

    @Override
    public final void update() {
        updateDrivebase();
    }


    private void updateDrivebase() {
        drivebase.state = driver.isXButtonDown() ? Drivebase.State.ZERO : Drivebase.State.FIELD_RELATIVE;

        updateDrivebaseSpeeds();
        updateDrivebaseAlign();
        updateDrivebasePositions();
    }

    private void updateDrivebaseAlign() {
        if (driver.isDPADUpDown()) 
            drivebase.setAlign(Field.AlignState.CENTER);
         else if (driver.isDPADLeftDown()) 
          drivebase.setAlign(Field.AlignState.LEFT);
         else if (driver.isDPADRightDown()) 
            drivebase.setAlign(Field.AlignState.RIGHT);
    }

    private boolean slowMode = false;
    private final TorqueClick slowModeClick = new TorqueClick();

    private final static double DEADBAND = 0.05;

    private void updateDrivebaseSpeeds() {
        if (slowModeClick.calculate(driver.isLeftBumperPressed())) slowMode = !slowMode;
        final double speedSetting = slowMode ? 0.2 : 1;
        
        final double xVelocity = TorqueMath
                .scaledDeadband(driver.getLeftYAxis() * Drivebase.MAX_VELOCITY * speedSetting, DEADBAND);
        final double yVelocity = TorqueMath
                .scaledDeadband(driver.getLeftXAxis() * Drivebase.MAX_VELOCITY * speedSetting, DEADBAND);
        final double rotationVelocity = TorqueMath
                .scaledDeadband(-driver.getRightXAxis() * Drivebase.MAX_ANGULAR_VELOCITY * speedSetting, DEADBAND);

        drivebase.inputSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);

        drivebase.requestedRotation = Math.PI + Math.atan2(driver.getRightXAxis(), driver.getRightYAxis());
        if (drivebase.requestedRotation == Math.PI)
            drivebase.requestedRotation = 0;
    }

    private final TorqueClick resetGyro = new TorqueClick();
    private final TorqueClick resetPose = new TorqueClick();

    private void updateDrivebasePositions() {
        if (resetGyro.calculate(driver.isRightCenterButtonPressed()))
            drivebase.resetGyro();

        if (resetPose.calculate(driver.isLeftCenterButtonPressed()))
            // drivebase.resetPose(new Translation2d(0, 0));
            drivebase.resetPose(Drivebase.INITIAL_POS);
    }


    private final TorqueClick toggleRotationLock = new TorqueClick();
    private final TorqueClick toggleSmartDrive = new TorqueClick();
    private boolean usingSmartDrive = false;
    private final TorqueClick toggleIsDirectRotation = new TorqueClick();

    private void updateDrivebaseSettings() {
        if (toggleSmartDrive.calculate(driver.isBButtonDown()))
            drivebase.setSmartDrive(usingSmartDrive = !usingSmartDrive);

        if (toggleRotationLock.calculate(driver.isAButtonDown()))
            drivebase.isRotationLocked = !drivebase.isRotationLocked;

        if (toggleIsDirectRotation.calculate(driver.isYButtonDown()))
            drivebase.isDirectRotation = !drivebase.isDirectRotation;

    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}