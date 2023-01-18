/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.Optional;

import org.texastorque.Subsystems;
import org.texastorque.controllers.AutoLevelController;
import org.texastorque.controllers.CameraController;
import org.texastorque.controllers.SwerveAlignmentController;
import org.texastorque.controllers.SwerveAlignmentController.AlignState;
import org.texastorque.controllers.SwerveAlignmentController.GridState;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.modules.TorqueSwerveModule2022;
import org.texastorque.torquelib.modules.TorqueSwerveModule2022.TorqueSwerveModuleConfiguration;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import io.github.oblarg.oblog.annotations.Log;

public final class Drivebase extends TorqueSubsystem implements Subsystems {
    private static volatile Drivebase instance;

    public static enum State {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), ALIGN(FIELD_RELATIVE), ZERO(FIELD_RELATIVE), BALANCE(FIELD_RELATIVE);

        public final State parent;
        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    public State state = State.ROBOT_RELATIVE;

    public static final double WIDTH = Units.inchesToMeters(21.745), // m (swerve to swerve)
            LENGTH = Units.inchesToMeters(21.745), // m (swerve to swerve)

            MAX_VELOCITY = 4.522, // m/s
            MAX_ACCELERATION = 8.958, // m/s^2
            MAX_ANGULAR_VELOCITY = 2 * Math.PI, // rad/s
            MAX_ANGULAR_ACCELERATION = 2 * Math.PI, // rad/s^2
            WHEEL_DIAMETER = Units.inchesToMeters(4.0), // m

            MAGIC_NUMBER = 34;

    public static final Pose2d INITIAL_POS = new Pose2d(0, 0, new Rotation2d(0));

    private static final double SIZE = Units.inchesToMeters(12);

    private final Translation2d
            LOC_FL = new Translation2d(SIZE, -SIZE),
            LOC_FR = new Translation2d(SIZE, SIZE),
            LOC_BL = new Translation2d(-SIZE, -SIZE),
            LOC_BR = new Translation2d(-SIZE, SIZE);

    // This is the kinematics object that calculates the desired wheel speeds
    private final SwerveDriveKinematics kinematics;

    // PoseEstimator is a more advanced odometry system that uses a Kalman filter to
    // estimate the robot's position
    // It also encorporates other measures like April tag positions
    private final SwerveDrivePoseEstimator poseEstimator;

    @Log.Field2d(name = "Robot Field")
    public final Field2d fieldMap = new Field2d();

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    
    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));


    private final TorqueSwerveModule2022 fl, fr, bl, br;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private double lastRotationRadians;
    private final ProfiledPIDController rotationalPID = new ProfiledPIDController(.025, .001, 0, SwerveAlignmentController.OMEGA_CONSTRAINTS), 
        directRotPID = new ProfiledPIDController(1, 0, 0, SwerveAlignmentController.OMEGA_CONSTRAINTS);

    private SwerveModuleState[] swerveStates;

    @Log.ToString(name = "Chassis Speeds")
    public ChassisSpeeds inputSpeeds = new ChassisSpeeds(0, 0, 0);

    public double requestedRotation = 0;
    public boolean isRotationLocked = true;

    private final SwerveAlignmentController alignmentController = new SwerveAlignmentController(
        this::getPose, () -> state = state.parent);

    public void setAlignState(final AlignState alignment) {
        state = alignment == AlignState.NONE ? state.parent : State.ALIGN;
        alignmentController.setAlignment(alignment);
    }

    public void setGridOverride(final GridState override) {
        alignmentController.setGridOverride(override);
    }

    private final CameraController cameraController = new CameraController();

    private final AutoLevelController autoLevelController = new AutoLevelController(this::getPose);

    private Drivebase() {
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
        lastRotationRadians = gyro.getRotation2d().getRadians();
        directRotPID.enableContinuousInput(0, 2 * Math.PI);

        final TorqueSwerveModuleConfiguration config = TorqueSwerveModuleConfiguration.defaultConfig;

        config.maxVelocity = MAX_VELOCITY;
        config.maxAcceleration = MAX_ACCELERATION;
        config.maxAngularVelocity = MAX_ANGULAR_VELOCITY;
        config.maxAngularAcceleration = MAX_ANGULAR_ACCELERATION;

        // Configure all the swerve modules Drive|Turn|Encoder|Offset
        fl = new TorqueSwerveModule2022("Front Left", 3, 4, 10, 5.769290082156658, config);
        fr = new TorqueSwerveModule2022("Front Right", 5, 6, 11, 4.312011279165745, config);
        bl = new TorqueSwerveModule2022("Back Left", 1, 2, 9, 1.135143488645554, config);
        br = new TorqueSwerveModule2022("Back Right", 7, 8, 12, 5.186378560960293, config);

        kinematics = new SwerveDriveKinematics(LOC_BL, LOC_BR, LOC_FL, LOC_FR);

        poseEstimator =  new SwerveDrivePoseEstimator(
                    kinematics,
                    gyro.getHeadingCCW(),
                    getModulePositions(),
                    INITIAL_POS,
                    STATE_STDS,
                    VISION_STDS);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();
    }

  
    @Override
    public final void initialize(final TorqueMode mode) {
        mode.onAuto(() -> {
            isRotationLocked = false;
            state = State.ROBOT_RELATIVE;
        });

        mode.onTeleop(() -> {
            isRotationLocked = true;
            state = State.FIELD_RELATIVE;
        });
    }

    public static SwerveModulePosition invertSwerveModuleDistance(final SwerveModulePosition pose) {
        return new SwerveModulePosition(-pose.distanceMeters, pose.angle);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { 
            invertSwerveModuleDistance(fl.getPosition()), 
            invertSwerveModuleDistance(fr.getPosition()), 
            invertSwerveModuleDistance(bl.getPosition()), 
            invertSwerveModuleDistance(br.getPosition()) 
        };
    }

    private void updateFeedback() {
        cameraController.update(poseEstimator::addVisionMeasurement);

        poseEstimator.update(gyro.getHeadingCCW(), getModulePositions());

        fieldMap.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    private void zeroModules() {
        fl.zero();
        fr.zero();
        bl.zero();
        br.zero();
    }

    private void preseveModulePositions() {
        fl.setDesiredState(new SwerveModuleState(0, swerveStates[0].angle));
        fr.setDesiredState(new SwerveModuleState(0, swerveStates[1].angle));
        bl.setDesiredState(new SwerveModuleState(0, swerveStates[2].angle));
        br.setDesiredState(new SwerveModuleState(0, swerveStates[3].angle));
    }

    private void calculateTeleop() {
        SmartDashboard.putBoolean("isRotLocked", isRotationLocked);
        final double realRotationRadians = gyro.getHeadingCCW().getRadians();

        if (isRotationLocked && inputSpeeds.omegaRadiansPerSecond == 0) {
            final double omega = rotationalPID.calculate(
                    realRotationRadians, lastRotationRadians);
            inputSpeeds.omegaRadiansPerSecond = omega;
        } else
            lastRotationRadians = realRotationRadians;
    
    }

    public void convertToFieldRelative() {
        inputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                inputSpeeds.vxMetersPerSecond,
                inputSpeeds.vyMetersPerSecond,
                inputSpeeds.omegaRadiansPerSecond,
                gyro.getHeadingCCW());
    }

    public void setSmartDrive(final boolean isSmart) {
        fl.useSmartDrive = isSmart;
        fr.useSmartDrive = isSmart;
        bl.useSmartDrive = isSmart;
        br.useSmartDrive = isSmart;
    }

    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();

        if (state == State.ZERO) {
            zeroModules();
        } else {
            if (state == State.ALIGN) {
                // setSmartDrive(true);
                final Optional<ChassisSpeeds> speedsWrapper = alignmentController.calculateAlignment();
                if (speedsWrapper.isPresent()) {
                    inputSpeeds = speedsWrapper.get();
                    convertToFieldRelative();
                }


                lights.set(Color.kGreen, Lights.OFF);
            } else if (state == State.BALANCE) {
                // setSmartDrive(true);
                inputSpeeds = autoLevelController.calculate();
                convertToFieldRelative();
                // autoLevelController.calculate();
            }


            
            if (state == State.FIELD_RELATIVE) {
                // setSmartDrive(false);
                calculateTeleop();
                lights.set(Lights.ALLIANCE, Lights.SOLID);
                convertToFieldRelative();
            }

            if (state != State.ROBOT_RELATIVE) {
            }

            swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

            fl.setDesiredState(swerveStates[0]);
            fr.setDesiredState(swerveStates[1]);
            bl.setDesiredState(swerveStates[2]);
            br.setDesiredState(swerveStates[3]);

            if (inputSpeeds.vxMetersPerSecond == 0 && inputSpeeds.vyMetersPerSecond == 0 && inputSpeeds.omegaRadiansPerSecond == 0) {
                preseveModulePositions();
            }
        }

        alignmentController.resetIf(state != State.ALIGN);
        autoLevelController.resetIf(state != State.BALANCE);
       
        state = state.parent;
    }

    public void resetPose(final Pose2d pose) {
        gyro.setOffsetCW(pose.getRotation());
        poseEstimator.resetPosition(gyro.getHeadingCCW(), getModulePositions(), pose);
    }

    public void resetPose(final Rotation2d rotation) {
        resetPose(new Pose2d(getPose().getTranslation(), rotation));
    }

    public void setAngle(final Rotation2d rotation) {
        gyro.setOffsetCW(rotation);
    }

    public void resetPose(final Translation2d translation) {
        resetPose(new Pose2d(translation, gyro.getHeadingCCW()));
    }

    public void resetGyro() {
        gyro.setOffsetCW(new Rotation2d(0));
    }

    public void setState(final State state) {
        this.state = state;
    }

    @Log.ToString(name = "Robot Pose")
    public Pose2d getPose() {
        updateFeedback();
        return poseEstimator.getEstimatedPosition();
    }

    @Log.ToString(name = "Robot Pose X")
    private double logPoseX() {
        return getPose().getTranslation().getX();
    }

    @Log.ToString(name = "Robot Pose Y")
    private double logPoseY() {
        return getPose().getTranslation().getY();
    }
    
    @Log.Dial(name = "Gyro Radians")
    public double getGyroAngle() {
        return gyro.getHeadingCCW().getRadians();
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }
}
