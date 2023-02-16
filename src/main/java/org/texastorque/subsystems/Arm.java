/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Hand.GamePiece;
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
        private static final double ELEVATOR_TOLERANCE = 1, ROTARY_TOLERANCE = (1. / 12.) * Math.PI;

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
        HANDOFF(new ArmPose(0, Rotation2d.fromDegrees(260))),
        DOWN(new ArmPose(0.25, Rotation2d.fromDegrees(235))),
        BACK(new ArmPose(0, Rotation2d.fromDegrees(200))),
        SHELF(new ArmPose(0, Rotation2d.fromDegrees(0))),            
        MID(
                new ArmPose(0, Rotation2d.fromDegrees(0)), 
                new ArmPose(.175, Rotation2d.fromDegrees(25))
        ), 
        TOP(
                new ArmPose(1,  Rotation2d.fromDegrees(0)), 
                new ArmPose(1.1,  Rotation2d.fromDegrees(20))
        );
     
        public final ArmPose cubePose;
        public final ArmPose conePose;

        private State(final ArmPose both) { this(both, both); }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public ArmPose get() { return hand.getGamePieceMode() == GamePiece.CUBE ? cubePose : conePose; }
    }

    private static final double ROTARY_ENCODER_OFFSET = -4.203098863363266, 
            ELEVATOR_MAX_VOLTS = 12,
            ROTARY_MAX_VOLTS = 8, 
            ELEVATOR_MIN = 0, 
            ELEVATOR_MAX = 1.3;

    private static volatile Arm instance;
    public static final synchronized Arm getInstance() { return instance == null ? instance = new Arm() : instance; }

    @Log.ToString
    private State activeState = State.DOWN;
    @Log.ToString
    private State desiredState = State.DOWN;
    @Log.ToString
    private State lastState = State.DOWN;
    @Log.ToString
    public double realElevatorPose = 0;

    @Log.ToString
    public Rotation2d realRotaryPose = Rotation2d.fromDegrees(0);
    private final TorqueNEO elevator = new TorqueNEO(Ports.ARM_ELEVATOR_MOTOR);
    @Config
    public final PIDController elevatorPoseController = new PIDController(15, 0, 0);

    // Not using rn
    private final ElevatorFeedforward elevatorPoseFeedForward = new ElevatorFeedforward(0, 0, 0);
    /**
     *    π/2
     *     |
     * π --+-- 0 (towards goal)
     *     |
     *   3π/2
     */

    private final TorqueNEO rotary = new TorqueNEO(Ports.ARM_ROTARY_MOTOR);

    @Config
    public final PIDController rotaryPoseController = new PIDController(.5 * Math.PI, 0 * Math.PI, 0 * Math.PI);

    // Unfortunatly these are not sendables
    // Est. from https://www.reca.lc/arm
    public final ArmFeedforward rotaryPoseFeedForward = new ArmFeedforward(0, .75, .5);
    // public final ArmFeedforward rotaryPoseFeedForward = new ArmFeedforward(0, 1.5, .5);

    private final TorqueCANCoder rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);

    @Log.BooleanBox
    private boolean wantsHandoff = false;

    private Arm() {
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

    // TODO: check if at pose
    @Log.BooleanBox
    public boolean isAtShelf() {
        return activeState == State.SHELF;
    }

    // TODO: check if at pose
    @Log.BooleanBox
    public boolean isAtScoringPose() {
        return activeState == State.MID || activeState == State.TOP;
    }
    public void setState(final State state) { this.desiredState = state; }

    public State getState() { return desiredState; }
    public boolean isState(final State state) { return getState() == state; }

    @Override
    public final void initialize(final TorqueMode mode) {

    }

    @Log.ToString
    public boolean isAtDesiredPose() { return activeState.get().atPose(realElevatorPose, realRotaryPose); }

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;
        if (hand.isState(Hand.State.OPEN) && activeState == State.DOWN)
            activeState = State.HANDOFF;

        updateFeedback();

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

        final double constrainedElevatorVolts = TorqueMath.linearConstraint(requestedElevatorVolts, realElevatorPose, ELEVATOR_MIN, ELEVATOR_MAX); // don't think this will work
        SmartDashboard.putNumber("arm::constrainedElevatorVolts", constrainedElevatorVolts);

        elevator.setVolts(constrainedElevatorVolts);
    }

    private void calculateRotary() {
        final double rotaryFFOutput = -rotaryPoseFeedForward.calculate(realRotaryPose.getRadians(), 0);

        final double rotarayPIDDOutput = -rotaryPoseController.calculate(realRotaryPose.getRadians(), activeState.get().rotaryPose.getRadians());
            
        final double requestedRotaryVolts = TorqueMath.constrain(rotarayPIDDOutput + rotaryFFOutput, ROTARY_MAX_VOLTS);
        SmartDashboard.putNumber("arm::requestedRotaryVolts", requestedRotaryVolts);

        rotary.setVolts(rotaryEncoder.isCANResponsive() ? requestedRotaryVolts : 0);
    }
}
