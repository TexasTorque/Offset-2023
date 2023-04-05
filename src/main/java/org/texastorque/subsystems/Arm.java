/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.sensors.TorqueCANCoder;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Arm extends TorqueSubsystem implements Subsystems {
    public static class ArmPose {
        private static final double ELEVATOR_TOLERANCE = .4, ROTARY_TOLERANCE = (1. / 12.) * Math.PI;

        public boolean autoReadyToScore = false;

        public final double elevatorPose;
        public final Rotation2d rotaryPose;

        public ArmPose(final double elevatorPose, final Rotation2d rotaryPose) {
            this.elevatorPose = elevatorPose;
            this.rotaryPose = rotaryPose;
        }

        public boolean atPose(final double elevatorReal, final Rotation2d rotaryReal) {
            return Math.abs(elevatorReal - elevatorPose) < ELEVATOR_TOLERANCE
                    && Math.abs(rotaryReal.minus(rotaryPose).getRadians()) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        // The order is cube, cone

        // Normal states
        SCORING_HALF_WAY_POINT(new ArmPose(0.45, Rotation2d.fromDegrees(90))),
        SHELF(new ArmPose(3, Rotation2d.fromDegrees(220)),
                new ArmPose(0, Rotation2d.fromDegrees(220))),
        STOWED(SHELF),
        MID(
                new ArmPose(0, Rotation2d.fromDegrees(0)),
                new ArmPose(5, Rotation2d.fromDegrees(20))),
        TOP(
                new ArmPose(30, Rotation2d.fromDegrees(0)),
                new ArmPose(43, Rotation2d.fromDegrees(10))),

        // Not really used states
        LOW(new ArmPose(.6, Rotation2d.fromDegrees(0))),
        THROW(new ArmPose(50, Rotation2d.fromDegrees(0))),

        PRIME(new ArmPose(0, Rotation2d.fromDegrees(120))),

        // Handoff related states
        HANDOFF(new ArmPose(20, Rotation2d.fromDegrees(220))),

        HANDOFF_ABOVE(
                new ArmPose(15, Rotation2d.fromDegrees(215)),
                new ArmPose(16, Rotation2d.fromDegrees(240))),
        HANDOFF_DOWN(
                new ArmPose(8, Rotation2d.fromDegrees(260)),
                new ArmPose(2, Rotation2d.fromDegrees(242))),
        HANDOFF_GRAB(
                new ArmPose(9, Rotation2d.fromDegrees(260)),
                new ArmPose(2, Rotation2d.fromDegrees(235))),
        HANDOFF_GRAB_BACK(
                new ArmPose(20, Rotation2d.fromDegrees(232))),
        HANDOFF_BACK(
                new ArmPose(6, Rotation2d.fromDegrees(230)),
                new ArmPose(16, Rotation2d.fromDegrees(230)));

        public final ArmPose cubePose;
        public final ArmPose conePose;

        private State(final ArmPose both) {
            this(both, both);
        }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        private State(final State other) {
            this(other.cubePose, other.conePose);
        }

        public ArmPose get() {
            return hand.isCubeMode() ? cubePose : conePose;
        }
    }

    public static class ConeHandoff extends TorqueSequence implements Subsystems {
        public ConeHandoff() {
            goTo(State.HANDOFF_ABOVE, .35);
            goTo(State.HANDOFF_DOWN, .25);
            goTo(State.HANDOFF_GRAB, .35);
            // goTo(State.HANDOFF_BACK, .25);
        }

        private final void goTo(final State state, final double seconds) {
            addBlock(new TorqueWaitUntil(() -> {
                arm.activeState = state;
                return arm.isAtState(state);
            }));

            if (seconds == -1)
                return;

            addBlock(new TorqueWaitForSeconds(seconds, () -> {
                arm.activeState = state;
            }));
        }
    }

    public static class CubeHandoff extends TorqueSequence implements Subsystems {
        public CubeHandoff() {
            goTo(State.HANDOFF_DOWN, .25);
            goTo(State.HANDOFF_GRAB, .35);
            goTo(State.HANDOFF_GRAB_BACK, .25);
        }

        private final void goTo(final State state, final double seconds) {
            addBlock(new TorqueWaitUntil(() -> {
                arm.activeState = state;
                return arm.isAtState(state);
            }));

            if (seconds == -1)
                return;

            addBlock(new TorqueWaitForSeconds(seconds, () -> {
                arm.activeState = state;
            }));
        }
    }

    private static final double ROTARY_ENCODER_OFFSET = Units.degreesToRadians(-65),
            ELEVATOR_MAX_VOLTS_UP = 12,
            ELEVATOR_MAX_VOLTS_HANDOFF = 12,
            ELEVATOR_MAX_VOLTS_DOWN = 6,
            ROTARY_MAX_VOLTS = 6,
            ELEVATOR_MIN = 0,
            ELEVATOR_MAX = 50;

    private static volatile Arm instance;

    public static final synchronized Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }

    private double setpointAdjustment = 0;

    @Log.ToString
    private State activeState = State.STOWED;
    @Log.ToString
    private State desiredState = State.STOWED;
    @Log.ToString
    private State lastState = State.STOWED;

    @Log.ToString
    public double realElevatorPose = 0;
    @Log.ToString
    public Rotation2d realRotaryPose = Rotation2d.fromDegrees(0);
    private final TorqueNEO elevator = new TorqueNEO(Ports.ARM_ELEVATOR_MOTOR);

    @Config
    public final PIDController elevatorPoseController = new PIDController(1.13, 0, 0);

    private final ElevatorFeedforward elevatorPoseFeedForward = new ElevatorFeedforward(0.072294, 0.28979, 0.12498,
            0.0019267);

    private final TorqueNEO rotary = new TorqueNEO(Ports.ARM_ROTARY_MOTOR);

    @Config
    public final PIDController rotaryPoseController = new PIDController(1.89, 0, 0);

    public final ArmFeedforward rotaryFeedforward = new ArmFeedforward(0.18362, 0.22356, 4, 4.4775);

    private final TorqueCANCoder rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);

    private ConeHandoff coneHandoff = new ConeHandoff();

    private CubeHandoff cubeHandoff = new CubeHandoff();

    private CubeHandoff autoCubeHandoff = new CubeHandoff();

    private Arm() {
        elevator.setCurrentLimit(30);
        elevator.setVoltageCompensation(12.6);
        elevator.setBreakMode(true);
        elevator.invertMotor(false);
        elevator.burnFlash();

        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);
    }

    public State getActiveState() {
        return activeState;
    }

    public boolean isStowed() {
        return activeState == State.STOWED;
    }

    public boolean isPerformingHandoff() {
        return desiredState == State.HANDOFF;
    }

    @Log.BooleanBox
    public boolean isWantingShelf() {
        return desiredState == State.SHELF;
    }

    @Log.BooleanBox
    public boolean isWantingScoringPose() {
        return desiredState == State.MID || desiredState == State.TOP;
    }

    @Log.BooleanBox
    public boolean isWantingHighCOG() {
        return isWantingScoringPose();
    }

    @Log.BooleanBox
    public boolean isAtState(final State state) {
        return activeState.get().atPose(realElevatorPose, realRotaryPose);
    }

    @Log.BooleanBox
    public boolean isAtScoringPose() {
        return isAtState(State.MID) || isAtState(State.TOP);
    }

    @Log.BooleanBox
    public boolean isAtShelf() {
        return isAtState(State.SHELF);
    }

    @Log.BooleanBox
    public boolean hasHighCOG() {
        return isAtScoringPose() || isAtShelf();
    }

    public void setState(final State state) {
        this.desiredState = state;
    }

    public void setActiveState(final State state) {
        this.activeState = state;
    }

    public State getState() {
        return desiredState;
    }

    public boolean isState(final State state) {
        return getState() == state;
    }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    @Override
    public final void initialize(final TorqueMode mode) {

    }

    @Log.ToString
    public boolean isAtDesiredPose() {
        return activeState.get().atPose(realElevatorPose, realRotaryPose);
    }

    public boolean isWantingHalfOpen() {
        return activeState == State.HANDOFF_ABOVE;

    }

    public boolean isWantingFullOpen() {
        return activeState == State.HANDOFF_DOWN;
    }

    public void setSetpointAdjustment(final double setpointAdjustment) {
        this.setpointAdjustment = setpointAdjustment;
        if (Math.abs(this.setpointAdjustment) < .1)
            this.setpointAdjustment = 0;
    }

    public boolean isDoingHandoff() {
        return activeState == State.HANDOFF;
    }

    @Override
    public final void update(final TorqueMode mode) {

        activeState = desiredState;

        updateFeedback();

        if (activeState == State.HANDOFF) {
            if (hand.isConeMode())
                coneHandoff.run();
            else if (mode.isAuto())
                autoCubeHandoff.run();
            else
                cubeHandoff.run();

        } else {
            activeState = desiredState;
            if (mode.isTeleop()) {
                coneHandoff = new ConeHandoff();
                cubeHandoff = new CubeHandoff();
            } else {
                autoCubeHandoff = new CubeHandoff();
            }

        }

        if (forks.isForksRunning()) activeState = State.SHELF;

        if (desiredState == State.THROW) {
            calculateElevator(State.THROW);
            calculateRotary(State.THROW);
            lastState = State.TOP;
        } else if (isComingDown() && !isElevatorDownEnough()) {
            calculateElevator(activeState);
            calculateRotary(State.SCORING_HALF_WAY_POINT);
        } else if (isGoingUp() && !isArmOutEnough()) {
            calculateElevator(State.STOWED);
            calculateRotary(activeState);
        } else {
            calculateElevator(activeState);
            calculateRotary(activeState);
            lastState = activeState;
        }

    }

    public boolean isReadyToThrow() {
        return desiredState == State.THROW && realRotaryPose.getDegrees() <= 145;
    }

    @Log.BooleanBox
    public boolean isComingDown() {
        return lastState == State.TOP && activeState != State.TOP && !isGoingUp();
    }

    @Log.BooleanBox
    public boolean isGoingUp() {
        return lastState != State.TOP && activeState == State.TOP;
    }

    private boolean isElevatorDownEnough() {
        return realElevatorPose <= 14;
    }

    private boolean isArmOutEnough() {
        return realRotaryPose.getRadians() <= Math.PI * .75;
    }

    private void updateFeedback() {
        realElevatorPose = elevator.getPosition();
        final double rotaryRadians = TorqueMath.constrain0to2PI(-rotaryEncoder.getPosition() - ROTARY_ENCODER_OFFSET);
        realRotaryPose = Rotation2d.fromRadians(rotaryRadians);
    }

    private void calculateElevator(final State state) {
        final double elevatorSetpoint = state.get().elevatorPose;
        double elevatorVolts = elevatorPoseController.calculate(realElevatorPose, elevatorSetpoint);
        elevatorVolts += elevatorPoseFeedForward.calculate(
                calculateElevatorVelocity(elevatorSetpoint, realElevatorPose),
                calculateElevatorAcceleration(elevatorSetpoint, realElevatorPose));
        final double maxVoltsNormal = elevatorVolts > 0 ? ELEVATOR_MAX_VOLTS_UP : ELEVATOR_MAX_VOLTS_DOWN;
        elevatorVolts = TorqueMath.constrain(elevatorVolts,
                isPerformingHandoff() ? ELEVATOR_MAX_VOLTS_HANDOFF : maxVoltsNormal);
        elevatorVolts = TorqueMath.linearConstraint(elevatorVolts, realElevatorPose, ELEVATOR_MIN, ELEVATOR_MAX);
        elevator.setVolts(elevatorVolts);
        Debug.log("elevatorCurrent", elevator.getCurrent());
        Debug.log("elevatorRequestedVolts", elevatorVolts);
    }

    // omega with respect to delta x
    private double calculateElevatorVelocity(final double wanted, final double actual) {
        return Math.signum(wanted - actual)
                * (80. / (1 + Math.pow(Math.E, -.05 * (Math.abs(wanted - actual) - 80. / 2))) - 9.53);
    }

    // derivative of calculateElevatorVelocity
    private double calculateElevatorAcceleration(final double wanted, final double actual) {
        return Math.signum(wanted - actual) * (4 * Math.pow(Math.E, -.05 * (wanted - actual - 40))
                / Math.pow(Math.pow(Math.E, -.05 * (wanted - actual - 40)) + 1, 2));
    }

    private void calculateRotary(final State state) {
        double armSetpoint = state.get().rotaryPose.getRadians() + setpointAdjustment * Units.degreesToRadians(30);

        double rotaryPos = realRotaryPose.getRadians();
        if (rotaryPos > Math.toRadians(315)) { // wrap around up to prevent overshoot causing a massive spin.
            rotaryPos = rotaryPos - 2 * Math.PI;
        }
        double rotaryVolts = -rotaryFeedforward.calculate(armSetpoint, calculateRotaryVelocity(armSetpoint, rotaryPos),
                calculateRotaryAcceleration(armSetpoint, rotaryPos));
        // final boolean stopArm = armSetpoint <= (Math.PI * 0.5) && armSwitch.get();
        rotaryVolts += -rotaryPoseController.calculate(rotaryPos, armSetpoint);
        rotaryVolts = TorqueMath.constrain(rotaryVolts, ROTARY_MAX_VOLTS);
        // rotary.setVolts(rotaryEncoder.isCANResponsive() && !isState(Arm.State.LOW) ?
        // rotaryVolts : 0);
        rotary.setVolts(rotaryVolts);
        Debug.log("rotaryVolts", rotaryVolts);
        Debug.log("elevatorCurrent", rotary.getCurrent());
        SmartDashboard.putBoolean("rotaryCANResponsiveness", rotaryEncoder.isCANResponsive());
    }

    // omega with respect to delta theta (radians)
    private double calculateRotaryVelocity(final double wanted, final double actual) {
        return Math.signum(wanted - actual)
                * (15 / (1 + Math.pow(Math.E, -.3 * (Math.abs(wanted - actual) - 12))) - .399);
    }

    // derivative of calculateRotaryVelocity
    private double calculateRotaryAcceleration(final double wanted, final double actual) {
        return Math.signum(wanted - actual) * (12 * Math.pow(Math.E, -1.5 * (Math.abs(wanted - actual - 12)))
                / Math.pow(Math.pow(Math.E, -1.5 * (wanted - actual - 1.3)) + 1, 2));
    }
}