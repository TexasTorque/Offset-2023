/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

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
        private static final double ELEVATOR_TOLERANCE = .4, ROTARY_TOLERANCE = Units.degreesToRadians(20);

        public boolean autoReadyToScore = false;

        public final double elevatorPose;
        public final Rotation2d rotaryPose;

        public ArmPose(final double elevatorPose, final Rotation2d rotaryPose) {
            this.elevatorPose = elevatorPose;
            this.rotaryPose = rotaryPose;
        }

        public boolean atPose(final double elevatorReal, final Rotation2d rotaryReal) {
            SmartDashboard.putNumber("atPose::elev.real", elevatorReal);
            SmartDashboard.putNumber("atPose::elev.req", elevatorPose);
            SmartDashboard.putNumber("atPose::elev.delta", elevatorReal - elevatorPose);
            SmartDashboard.putNumber("atPose::rot.real", rotaryReal.getDegrees());
            SmartDashboard.putNumber("atPose::rot.req", rotaryPose.getDegrees());
            SmartDashboard.putNumber("atPose::rot.delta", rotaryReal.minus(rotaryPose).getDegrees());
            return Math.abs(elevatorReal - elevatorPose) < ELEVATOR_TOLERANCE
                    && Math.abs(rotaryReal.minus(rotaryPose).getRadians()) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        GRAB(
                new ArmPose(8, Rotation2d.fromDegrees(260)),
                new ArmPose(2, Rotation2d.fromDegrees(249))),
        AUTOGRAB(
                new ArmPose(5, Rotation2d.fromDegrees(266)),
                new ArmPose(0, Rotation2d.fromDegrees(180))),
        AUTOINDEX(
                new ArmPose(5, Rotation2d.fromDegrees(250)),
                new ArmPose(0, Rotation2d.fromDegrees(180))),
        INDEX(
                new ArmPose(15, Rotation2d.fromDegrees(215)),
                new ArmPose(16, Rotation2d.fromDegrees(240))),
        WAYPOINT(new ArmPose(0.45, Rotation2d.fromDegrees(90))),
        STOWED(new ArmPose(0, Rotation2d.fromDegrees(220))),
        SHELF(new ArmPose(10, Rotation2d.fromDegrees(220)),
                new ArmPose(0, Rotation2d.fromDegrees(220))),
        MID(
                new ArmPose(0 + 5, Rotation2d.fromDegrees(0)),
                new ArmPose(5 + 0, Rotation2d.fromDegrees(20))),
        TOP(
                new ArmPose(30 + 5, Rotation2d.fromDegrees(0)),
                new ArmPose(43 + 0, Rotation2d.fromDegrees(35))),
        LOW(new ArmPose(.6, Rotation2d.fromDegrees(0))),
        THROW(new ArmPose(50, Rotation2d.fromDegrees(0))),

        // Handoff waypoints
        HANDOFF(new ArmPose(0, Rotation2d.fromDegrees(180))),
        HANDOFF_ABOVE(
                new ArmPose(15, Rotation2d.fromDegrees(215)),
                new ArmPose(16, Rotation2d.fromDegrees(240))),
        HANDOFF_DOWN(
                new ArmPose(8, Rotation2d.fromDegrees(260)),
                new ArmPose(2, Rotation2d.fromDegrees(249))),
        HANDOFF_GRAB(HANDOFF_DOWN),
        HANDOFF_STOWED(
                new ArmPose(10, Rotation2d.fromDegrees(220)),
                new ArmPose(10, Rotation2d.fromDegrees(220)));

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

    public static class Handoff extends TorqueSequence implements Subsystems {
        public Handoff() {
            goTo(State.HANDOFF_ABOVE);
            wait(.25);
            goTo(State.HANDOFF_DOWN);
            wait(.25);
            goTo(State.HANDOFF_GRAB);
            wait(.25);
            goTo(State.HANDOFF_STOWED);
            wait(.25);
            goTo(State.STOWED);
        }

        private final void goTo(final State state) {
            addBlock(new TorqueWaitUntil(() -> {
                arm.activeState = state;

                return arm.isAtState(state);
            }));
        }

        private final void wait(final double seconds) {
            addBlock(new TorqueWaitForSeconds(seconds));
        }

    }

    private static final double ROTARY_ENCODER_OFFSET = -4.692437745630741,
            ELEVATOR_MAX_VOLTS_UP = 12,
            ELEVATOR_MAX_VOLTS_DOWN = 8,
            ROTARY_MAX_VOLTS_BACK = 6,
            ROTARY_MAX_VOLTS = 4,
            ELEVATOR_MIN = 0,
            ELEVATOR_MAX = 50; // 54 is the technical max

    private static volatile Arm instance;

    public static final synchronized Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }

    public boolean runArm = false;
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

    // @Config
    // public final PIDController rotaryPoseController = new PIDController(.5 *
    // Math.PI, 0 * Math.PI, 0 * Math.PI);

    // public final ArmFeedforward rotaryFeedforward = new ArmFeedforward(0.18362,
    // 0.22356, 4, 4.4775);

    private final TorqueNEO rotary = new TorqueNEO(Ports.ARM_ROTARY_MOTOR);

    public final PIDController rotaryPoseController = new PIDController(1.89 / 4, 0, 0);

    public final ArmFeedforward rotaryFeedforward = new ArmFeedforward(0.18362, 0.22356, 4, 4.4775);

    private final TorqueCANCoder rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);

    @Log.BooleanBox
    private boolean grabbing = false;

    public double setpointAdjustment = 0;

    private Handoff handoff;

    private Arm() {
        elevator.setCurrentLimit(30);
        elevator.setVoltageCompensation(12.6);
        elevator.setBreakMode(true);
        elevator.invertMotor(true);
        elevator.setConversionFactors(1, 1);
        elevator.burnFlash();

        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        // rotary.setConversionFactors(1, 1);
        rotary.burnFlash();

        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);

        activeState = State.STOWED;
    }

    public boolean isStowed() {
        return activeState == State.STOWED;
    }

    public boolean isPerformingHandoff() {
        return activeState == State.GRAB || activeState == State.INDEX;
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
        final boolean b = state.get().atPose(realElevatorPose, realRotaryPose);
        SmartDashboard.putBoolean("isAtState", b);
        return b;
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
        return false;
        // return activeState.get().atPose(realElevatorPose, realRotaryPose);
    }

    public boolean isWantGrabbyClaw() {
        return activeState == State.HANDOFF_GRAB || activeState == State.HANDOFF_DOWN;

    }

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        updateFeedback();

        if (handoff == null)
            handoff = new Handoff();

        // if (activeState == State.HANDOFF) {
        // handoff.run();
        // } else {
        // handoff.reset();
        // }

        calculateElevator();
        calculateRotary();

        lastState = activeState;
    }

    public void setSetpointAdjustment(final double setpointAdjustment) {
        this.setpointAdjustment = setpointAdjustment;
        if (Math.abs(this.setpointAdjustment) < .1)
            this.setpointAdjustment = 0;
    }

    private void updateFeedback() {
        realElevatorPose = -elevator.getPosition();
        final double rotaryRadians = TorqueMath.constrain0to2PI(-rotaryEncoder.getPosition() - ROTARY_ENCODER_OFFSET);
        realRotaryPose = Rotation2d.fromRadians(rotaryRadians);
        SmartDashboard.putNumber("rotaryCanCoder", realRotaryPose.getRadians());
    }

    private void calculateElevator() {
        final boolean isComingDown = (lastState == State.TOP || lastState == State.MID)
                && (activeState != State.TOP || activeState != State.MID);
        double elevatorVolts = elevatorPoseController.calculate(realElevatorPose, activeState.get().elevatorPose);
        elevatorVolts += elevatorPoseFeedForward.calculate(
                calculateElevatorVelocity(activeState.get().elevatorPose, realElevatorPose),
                calculateElevatorAcceleration(activeState.get().elevatorPose, realElevatorPose));
        elevatorVolts = TorqueMath.constrain(elevatorVolts,
                isComingDown ? ELEVATOR_MAX_VOLTS_UP : ELEVATOR_MAX_VOLTS_DOWN);
        elevatorVolts = TorqueMath.linearConstraint(elevatorVolts, realElevatorPose, ELEVATOR_MIN, ELEVATOR_MAX);
        elevator.setVolts(-elevatorVolts);

        SmartDashboard.putNumber("arm::elevatorCurrent", elevator.getCurrent());
        SmartDashboard.putNumber("arm::elevatorRequestedVolts", elevatorVolts);
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

    private void calculateRotary() {
        double armSetpoint = activeState.get().rotaryPose.getRadians();
        double rotaryPos = realRotaryPose.getRadians();

        if (rotaryPos > Math.toRadians(315)) {
            rotaryPos = rotaryPos - 2 * Math.PI;
        }
        double rotaryVolts = -rotaryFeedforward.calculate(armSetpoint, calculateRotaryVelocity(armSetpoint, rotaryPos),
                calculateRotaryAcceleration(armSetpoint, rotaryPos));
        // final boolean stopArm = armSetpoint <= (Math.PI * 0.5) && armSwitch.get();
        rotaryVolts += -rotaryPoseController.calculate(rotaryPos, armSetpoint);
        rotaryVolts = TorqueMath.constrain(rotaryVolts, ROTARY_MAX_VOLTS);
        // rotary.setVolts(rotaryEncoder.isCANResponsive() && !isState(Arm.State.LOW) ?
        // rotaryVolts : 0);

        // if (rotaryPos > Math.toRadians(315)) { // wrap around up to prevent overshoot
        // causing a massive spin.
        // rotaryVolts = -2;
        // }
        rotary.setVolts(rotaryVolts);

        SmartDashboard.putNumber("arm::rotaryVolts", rotaryVolts);
        SmartDashboard.putNumber("arm::elevatorCurrent", rotary.getCurrent());
        SmartDashboard.putNumber("arm::cancoder", rotaryEncoder.getPosition());

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
