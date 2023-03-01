/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.Optional;

import org.texastorque.Field;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.controllers.AutoLevelController;
import org.texastorque.controllers.PathAlignController;
import org.texastorque.controllers.PathAlignController.AlignState;
import org.texastorque.controllers.PathAlignController.GridState;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.sensors.TorqueVision;
import org.texastorque.torquelib.swerve.TorqueSwerveModule2022;
import org.texastorque.torquelib.swerve.TorqueSwerveModule2022.SwerveConfig;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import io.github.oblarg.oblog.annotations.Log;

public final class Drivebase extends TorqueSubsystem implements Subsystems {
    public static enum State {
        FIELD_RELATIVE(null),
        ROBOT_RELATIVE(null),
        ALIGN(FIELD_RELATIVE),
        ZERO(FIELD_RELATIVE),
        BALANCE(FIELD_RELATIVE);

        public final State parent;

        private State(final State parent) { this.parent = parent == null ? this : parent; }
    }

    public enum SpeedSetting {
        FAST(.8), SLOW(.25);

        public final double speed;
        private SpeedSetting(final double speed) {
            this.speed = speed;
        }
    }

    private static volatile Drivebase instance;
    
    public static final double WIDTH = Units.inchesToMeters(21.745), // m (swerve to swerve)
            LENGTH = Units.inchesToMeters(21.745),                   // m (swerve to swerve)

            MAX_VELOCITY = 4.522,                       // m/s
            MAX_ACCELERATION = 8.958,                   // m/s^2
            MAX_ANGULAR_VELOCITY = 2 * Math.PI,         // rad/s
            MAX_ANGULAR_ACCELERATION = 2 * Math.PI,     // rad/s^2
            WHEEL_DIAMETER = Units.inchesToMeters(4.0); // m

    public static final Pose2d INITIAL_POS = new Pose2d(0, 0, Rotation2d.fromRadians(0));

    private static final double SIZE = Units.inchesToMeters(18);

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in meters and radians, then meters.
     */
    private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(20));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision less. This matrix is in the form
     * [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));

    private static final Transform3d LEFT_CAMERA_TO_CENTER =
            new Transform3d(new Translation3d(Units.inchesToMeters(10.5), Units.inchesToMeters(7.5), Units.inchesToMeters(32.5625)),
                    new Rotation3d(0, 0, 0));

    private static final Transform3d RIGHT_CAMERA_TO_CENTER =
            new Transform3d(new Translation3d(Units.inchesToMeters(8.5), -Units.inchesToMeters(7.5), Units.inchesToMeters(21.5)), 
                    new Rotation3d(0, 0, 0));

    public static SwerveModulePosition invertSwerveModuleDistance(final SwerveModulePosition pose) {
        return new SwerveModulePosition(-pose.distanceMeters, pose.angle);
    }

    public static synchronized final Drivebase getInstance() { return instance == null ? instance = new Drivebase() : instance; }

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    @Log.ToString
    private State state = State.ROBOT_RELATIVE;

    private State requestedState = State.ROBOT_RELATIVE;

    private final Translation2d LOC_FL = new Translation2d(SIZE, -SIZE), LOC_FR = new Translation2d(SIZE, SIZE), LOC_BL = new Translation2d(-SIZE, -SIZE),
                                LOC_BR = new Translation2d(-SIZE, SIZE);
    // This is the kinematics object that calculates the desired wheel speeds
    private final SwerveDriveKinematics kinematics;

    // PoseEstimator is a more advanced odometry system that uses a Kalman
    // filter to estimate the robot's position It also encorporates other
    // measures like April tag positions
    private final SwerveDrivePoseEstimator poseEstimator;

    // @Log.Field2d(name = "Robot Field")
    public final Field2d fieldMap = new Field2d();

    private final TorqueSwerveModule2022 fl, fr, bl, br;
    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private double lastRotationRadians;

    private final PIDController teleopOmegaController = new PIDController(2 * Math.PI, 0, 0);

    private SwerveModuleState[] swerveStates;

    @Log.ToString(name = "Chassis Speeds")
    public TorqueSwerveSpeeds inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

    public double requestedRotation = 0;

    public boolean isRotationLocked = true;

    public final PathAlignController alignmentController = new PathAlignController(this::getPose, () -> inputSpeeds);

    private final AutoLevelController autoLevelController = new AutoLevelController(this::getPose);

    public final TorqueVision cameraLeft, cameraRight;
    public boolean updateWithTags = true;
    /**
     * Constructor called on initialization.
     */
    private Drivebase() {
        // Do this for each camera
        cameraLeft = new TorqueVision("camera_left", Field.getCurrentFieldLayout(), LEFT_CAMERA_TO_CENTER);
        cameraRight = new TorqueVision("camera_right", Field.getCurrentFieldLayout(), RIGHT_CAMERA_TO_CENTER);

        teleopOmegaController.enableContinuousInput(-Math.PI, Math.PI);
        lastRotationRadians = gyro.getRotation2d().getRadians();

        final SwerveConfig config = SwerveConfig.defaultConfig;

        config.maxVelocity = MAX_VELOCITY;
        config.maxAcceleration = MAX_ACCELERATION;
        config.maxAngularVelocity = MAX_ANGULAR_VELOCITY;
        config.maxAngularAcceleration = MAX_ANGULAR_ACCELERATION;

        // fl = new TorqueSwerveModule2022("Front Left", Ports.FL_MOD, 5.769290082156658, config);
        // fr = new TorqueSwerveModule2022("Front Right", Ports.FR_MOD, 4.312011279165745, config);
        // bl = new TorqueSwerveModule2022("Back Left", Ports.BL_MOD, 1.135143488645554, config);
        // br = new TorqueSwerveModule2022("Back Right", Ports.BR_MOD, 5.186378560960293, config);

        fl = new TorqueSwerveModule2022("Front Left", Ports.FL_MOD, -.53686679, config);
        fr = new TorqueSwerveModule2022("Front Right", Ports.FR_MOD, 1.365240141749382+Math.PI, config);
        bl = new TorqueSwerveModule2022("Back Left", Ports.BL_MOD, 1.135143488645554, config);
        br = new TorqueSwerveModule2022("Back Right", Ports.BR_MOD, 2.069323200489386 + Math.PI, config);

        kinematics = new SwerveDriveKinematics(LOC_BL, LOC_BR, LOC_FL, LOC_FR);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getHeadingCCW(), getModulePositions(), INITIAL_POS, STATE_STDS, VISION_STDS);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++) swerveStates[i] = new SwerveModuleState();
    }
    public void setState(final State state) { this.state = state; }
    public State getState() { return requestedState; }
    public boolean isState(final State state) { return getState() == state; }

    public void setAlignState(final AlignState alignment) {
        state = alignment == AlignState.NONE ? state.parent : State.ALIGN;
        alignmentController.setAlignment(alignment);
    }

    public void setGridOverride(final GridState override) { alignmentController.setGridOverride(override); }

    public final boolean isPathAlignDone() { return alignmentController.isDone(); } 

    public final boolean isAutoLevelDone() { return autoLevelController.isDone(); }

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
        updateWithTags = true;
        cameraLeft.setFieldLayout(Field.getCurrentFieldLayout());
        cameraRight.setFieldLayout(Field.getCurrentFieldLayout());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {invertSwerveModuleDistance(fl.getPosition()), invertSwerveModuleDistance(fr.getPosition()),
                                           invertSwerveModuleDistance(bl.getPosition()), invertSwerveModuleDistance(br.getPosition())};
    }

    public void convertToFieldRelative() { inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(gyro.getHeadingCCW()); } 

    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();
        requestedState = state;

        if (state == State.ZERO) {
            zeroModules();
        } else {
            if (arm.isWantingHighCOG())
                speedSetting = SpeedSetting.SLOW;

            if (state == State.ALIGN) {
                final Optional<TorqueSwerveSpeeds> speedsWrapper = alignmentController.calculate();
                if (speedsWrapper.isPresent()) inputSpeeds = speedsWrapper.get();

            } else if (state == State.BALANCE) {
                inputSpeeds = autoLevelController.calculate();
                convertToFieldRelative();
            }

            if (mode.isTeleop())
                inputSpeeds = inputSpeeds.times(speedSetting.speed);

            if (state == State.FIELD_RELATIVE) {
                calculateTeleop();
                convertToFieldRelative();
            }

            if (state != State.ROBOT_RELATIVE) {}

            swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

            if (inputSpeeds.hasZeroVelocity()) {
                preseveModulePositions();
            } else {

                fl.setDesiredState(swerveStates[0]);
                fr.setDesiredState(swerveStates[1]);
                bl.setDesiredState(swerveStates[2]);
                br.setDesiredState(swerveStates[3]);
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

    public void resetPose(final Rotation2d rotation) { resetPose(new Pose2d(getPose().getTranslation(), rotation)); } 

    public void setAngle(final Rotation2d rotation) { gyro.setOffsetCW(rotation); }

    public void resetPose(final Translation2d translation) { resetPose(new Pose2d(translation, gyro.getHeadingCCW())); }

    public void resetGyro() { gyro.setOffsetCW(Rotation2d.fromRadians(0)); }

    @Log.ToString(name = "Robot Pose")
    public Pose2d getPose() {
        updateFeedback();
        return poseEstimator.getEstimatedPosition();
    }

    @Log.Dial(name = "Gyro Radians")
    public double getGyroAngle() {
        return gyro.getHeadingCCW().getRadians();
    }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueContinuous(() -> setState(state));
    }

    public boolean isNotMoving() {
        return inputSpeeds.hasZeroVelocity();
    }

    private void updateFeedback() {
        if (updateWithTags) {
            cameraLeft.updateVisionMeasurement(poseEstimator::addVisionMeasurement);
            cameraRight.updateVisionMeasurement(poseEstimator::addVisionMeasurement);
        }

        poseEstimator.update(gyro.getHeadingCCW(), getModulePositions());

        fieldMap.setRobotPose(DriverStation.getAlliance() == DriverStation.Alliance.Blue ? poseEstimator.getEstimatedPosition()
                                                                                         : Field.reflectPosition(poseEstimator.getEstimatedPosition()));
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
        final double realRotationRadians = gyro.getHeadingCCW().getRadians();

        if (isRotationLocked && !inputSpeeds.hasRotationalVelocity() && inputSpeeds.hasTranslationalVelocity()) {
            final double omega = teleopOmegaController.calculate(realRotationRadians, lastRotationRadians);
            inputSpeeds.omegaRadiansPerSecond = omega;
        } else
            lastRotationRadians = realRotationRadians;
    }

    @Log.ToString(name = "Robot Pose X")
    private double logPoseX() {
        return getPose().getTranslation().getX();
    }

    @Log.ToString(name = "Robot Pose Y")
    private double logPoseY() {
        return getPose().getTranslation().getY();
    }
}
