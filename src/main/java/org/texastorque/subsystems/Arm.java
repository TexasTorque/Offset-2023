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
import org.texastorque.torquelib.auto.commands.TorqueExecute;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Arm extends TorqueSubsystem implements Subsystems {
    public static class ArmPose {
        private static final double ELEVATOR_TOLERANCE = .1, ROTARY_TOLERANCE = (1. / 12.) * Math.PI;

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
        GRAB(new ArmPose(0, Rotation2d.fromDegrees(260))),
        HANDOFF(new ArmPose(0.35, Rotation2d.fromDegrees(250))),

        BACK(new ArmPose(.3, Rotation2d.fromDegrees(230))),
        SHELF(new ArmPose(.65, Rotation2d.fromDegrees(0))),            
        MID(
                new ArmPose(.1, Rotation2d.fromDegrees(0)), 
                new ArmPose(.275, Rotation2d.fromDegrees(25))
        ), 
        TOP(
                new ArmPose(1.1,  Rotation2d.fromDegrees(0)), 
                new ArmPose(1.2,  Rotation2d.fromDegrees(20))
        );
     
        public final ArmPose cubePose;
        public final ArmPose conePose;

        private State(final ArmPose both) { this(both, both); }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public ArmPose get() { return hand.isCubeMode() ? cubePose : conePose; }
    }

    private static final double ROTARY_ENCODER_OFFSET = -4.203098863363266, 
            ELEVATOR_MAX_VOLTS = 12,
            ROTARY_MAX_VOLTS = 8, 
            ELEVATOR_MIN = 0, 
            ELEVATOR_MAX = 1.3;

    private static volatile Arm instance;
    public static final synchronized Arm getInstance() { return instance == null ? instance = new Arm() : instance; }

    @Log.ToString
    private State activeState = State.BACK;
    @Log.ToString
    private State desiredState = State.BACK;
    @Log.ToString
    private State lastState = State.BACK;
    @Log.ToString
    public double realElevatorPose = 0;

    @Log.ToString
    public Rotation2d realRotaryPose = Rotation2d.fromDegrees(0);
    private final TorqueNEO elevator = new TorqueNEO(Ports.ARM_ELEVATOR_MOTOR);
    @Config
    public final PIDController elevatorPoseController = new PIDController(15, 0, 0);

    private final ElevatorFeedforward elevatorPoseFeedForward = new ElevatorFeedforward(0, 0, 0);

    private final TorqueNEO rotary = new TorqueNEO(Ports.ARM_ROTARY_MOTOR);

    @Config
    public final PIDController rotaryPoseController = new PIDController(.5 * Math.PI, 0 * Math.PI, 0 * Math.PI);

    public final ArmFeedforward HIGH_COG_ROTARY_POSE_FEEDFORWARD = new ArmFeedforward(0, 1.25, 1);
    public final ArmFeedforward STANDARD_ROTARY_POSE_FEEDFORWARD = new ArmFeedforward(0, .75, .5);
    public ArmFeedforward currentRotaryPoseFeedForward;

    private final TorqueCANCoder rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);

    @Log.BooleanBox
    private boolean wantsHandoff = false;

    private State handoffState = State.HANDOFF;

    private Arm() {
        currentRotaryPoseFeedForward = STANDARD_ROTARY_POSE_FEEDFORWARD;

        elevator.setCurrentLimit(30);
        elevator.setVoltageCompensation(12.6);
        elevator.setBreakMode(true);
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

        activeState = State.BACK;
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
        return isWantingScoringPose() || isWantingShelf();
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

    public void setState(final State state) { this.desiredState = state; }
    
    public State getState() { return desiredState; }

    public boolean isState(final State state) { return getState() == state; }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    @Override
    public final void initialize(final TorqueMode mode) {

    }

    @Log.ToString
    public boolean isAtDesiredPose() { return activeState.get().atPose(realElevatorPose, realRotaryPose); }

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        updateFeedback();

        if (desiredState == State.HANDOFF) {
            if (handoffState == State.HANDOFF && isAtDesiredPose())
                handoffState = State.GRAB;
            
            activeState = handoffState;
        } else {
            handoffState = State.HANDOFF;
        } 

        calculateElevator();
        calculateRotary();

        lastState = activeState;
    }

    private void updateFeedback() {
        realElevatorPose = elevator.getPosition();

        final double rotaryRadians = TorqueMath.constrain0to2PI(-rotaryEncoder.getPosition() - ROTARY_ENCODER_OFFSET);
        realRotaryPose = Rotation2d.fromRadians(rotaryRadians);
    }

    private void calculateElevator() {
        final double elevatorPIDOutput = elevatorPoseController.calculate(realElevatorPose, activeState.get().elevatorPose);
        SmartDashboard.putNumber("arm::elevatorPIDOutput", elevatorPIDOutput);

        SmartDashboard.putNumber("arm::elevatorCurrent", elevator.getCurrent());

        final double elevatorFFOutput = elevatorPoseFeedForward.calculate(activeState.get().elevatorPose, 0);
        SmartDashboard.putNumber("arm::elevatorFFOutput", elevatorFFOutput);

        final double requestedElevatorVolts = TorqueMath.constrain(elevatorPIDOutput + elevatorFFOutput, ELEVATOR_MAX_VOLTS);
        SmartDashboard.putNumber("arm::requestedElevatorVolts", requestedElevatorVolts);

        final double constrainedElevatorVolts = TorqueMath.linearConstraint(requestedElevatorVolts, realElevatorPose, ELEVATOR_MIN, ELEVATOR_MAX); 
        SmartDashboard.putNumber("arm::constrainedElevatorVolts", constrainedElevatorVolts);

        elevator.setVolts(constrainedElevatorVolts);
    }

    private void calculateRotary() {
        currentRotaryPoseFeedForward = hand.isConeMode() && isWantingHighCOG() ? HIGH_COG_ROTARY_POSE_FEEDFORWARD : STANDARD_ROTARY_POSE_FEEDFORWARD;

        final double rotaryFFOutput = -currentRotaryPoseFeedForward.calculate(realRotaryPose.getRadians(), 0);

        final double rotarayPIDDOutput = -rotaryPoseController.calculate(realRotaryPose.getRadians(), activeState.get().rotaryPose.getRadians());
            
        final double requestedRotaryVolts = TorqueMath.constrain(rotarayPIDDOutput + rotaryFFOutput, ROTARY_MAX_VOLTS);
        SmartDashboard.putNumber("arm::requestedRotaryVolts", requestedRotaryVolts);

        rotary.setVolts(rotaryEncoder.isCANResponsive() ? requestedRotaryVolts : 0);
    }
}
