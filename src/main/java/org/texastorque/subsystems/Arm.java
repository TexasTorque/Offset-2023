/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

public final class Arm extends TorqueSubsystem implements Subsystems {
    private static volatile Arm instance;

    public static class ArmPose {
        private static final double ELEVATOR_TOLERANCE = 0.1, ROTARY_TOLERANCE = 0.1;

        public final double elevatorPose, rotaryPose;

        public ArmPose(final double elevatorPose, final double rotaryPose) {
            this.elevatorPose = elevatorPose;
            this.rotaryPose = rotaryPose;
        }

        public boolean atPose(final double elevatorReal, final double rotaryReal) {
            return Math.abs(elevatorReal - elevatorPose) < ELEVATOR_TOLERANCE && Math.abs(rotaryReal - rotaryPose) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        HANDOFF(new ArmPose(0, 0)),
        // Position to grab from indexer (contacts indexer)
        DOWN(new ArmPose(0, 0)),
        // Position to grab from ground (contacts ground)
        SHELF(new ArmPose(0, 0)),                  // Position to grab from shelf (contacts shelf)
        MID(new ArmPose(0, 0), new ArmPose(0, 0)), // Position to grab from human player (contacts
        // human player)
        TOP(new ArmPose(0, 0), new ArmPose(0, 0)); // Position to intake (contacts intake)

        public final ArmPose cubePose;
        public final ArmPose conePose;

        private State(final ArmPose both) { this(both, both); }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public ArmPose get() { return hand.getGamePieceMode() == GamePiece.CUBE ? cubePose : conePose; }
    }

    @Log.BooleanBox
    public boolean isAtShelf() {
        return activeState == State.SHELF;
    }

    @Log.BooleanBox
    public boolean isAtScoringPose() {
        return activeState == State.MID || activeState == State.TOP;
    }

    @Log.ToString
    private State activeState = State.HANDOFF;
    @Log.ToString
    private State desiredState = State.HANDOFF;

    public void setState(final State state) { this.desiredState = state; }

    public State getState() { return desiredState; }

    public boolean isState(final State state) { return getState() == state; }

    @Log.ToString
    public double realElevatorPose = 0;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO elevator = new TorqueNEO(Ports.ARM_ELEVATOR_MOTOR);
    @Config
    public final PIDController elevatorPoseController = new PIDController(0.1, 0, 0);
    // Not using rn
    private final ElevatorFeedforward elevatorPoseFeedForward = new ElevatorFeedforward(0, 0, 0);
    // new ElevatorFeedforward(1.01, 6.14, 0.16);


    /**
     *    π/2
     *     |
     * π --+-- 0 (towards goal)
     *     |
     *   3π/2
     */

    private final TorqueNEO rotary = new TorqueNEO(Ports.ARM_ROTARY_MOTOR);
    @Config
    public final PIDController rotaryPoseController = new PIDController(0.1, 0, 0);
    // Unfortunatly these are not sendables
    // Est. from https://www.reca.lc/arm
    public final ArmFeedforward rotaryPoseFeedForward = new ArmFeedforward(0, 4.25, 0, 0);
    // new ArmFeedforward(0, 4.25, 0.49, .2);

    private final CANCoder rotaryEncoder = new CANCoder(Ports.ARM_ROTARY_ENCODER);
    private static final double ARM_ROTARY_ENCODER_OFFSET = 0;

    private static double ELEVATOR_MOTOR_ROT_PER_METER = 0;

    private Arm() {
        elevator.setPositionConversionFactor(ELEVATOR_MOTOR_ROT_PER_METER);
        elevator.setCurrentLimit(20);
        elevator.setVoltageCompensation(12.6);
        elevator.setBreakMode(true);
        elevator.burnFlash();

        rotary.setCurrentLimit(20);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);

        activeState = State.DOWN;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Log.BooleanBox
    private boolean wantsHandoff = false;

    public boolean isAtDesiredPose() { return activeState.get().atPose(realElevatorPose, realRotaryPose); }

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        wantsHandoff = activeState == State.HANDOFF;
        if (wantsHandoff && indexer.isConflictingWithArm()) activeState = State.DOWN;

        realElevatorPose = elevator.getVelocity();
        realRotaryPose = rotaryEncoder.getPosition() - ARM_ROTARY_ENCODER_OFFSET;

        final double elevatorPIDOutput = elevatorPoseController.calculate(realElevatorPose, activeState.get().elevatorPose);
        SmartDashboard.putNumber("arm::elevatorPIDOutput", elevatorPIDOutput);

        final double elevatorFFOutput = elevatorPoseFeedForward.calculate(activeState.get().elevatorPose, 0);
        SmartDashboard.putNumber("arm::elevatorFFOutput", elevatorFFOutput);

        final double requestedElevatorVolts = elevatorPIDOutput + elevatorFFOutput;
        SmartDashboard.putNumber("arm::requestedElevatorVolts", requestedElevatorVolts);

        final double constrainedElevatorVolts = TorqueMath.linearConstraint(requestedElevatorVolts, realElevatorPose, ELEVATOR_MIN, ELEVATOR_MAX);
        SmartDashboard.putNumber("arm::constrainedElevatorVolts", constrainedElevatorVolts);

        // elevator.setVolts(constrainedElevatorVolts);

        final double rotaryFFOutput = rotaryPoseFeedForward.calculate(activeState.get().rotaryPose, 0);

        final double rotarayPIDDOutput = rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose);

        final double requestedRotaryVolts = rotarayPIDDOutput + rotaryFFOutput;
        SmartDashboard.putNumber("arm::requestedRotaryVolts", requestedRotaryVolts);

        // rotary.setVolts(requestedRotaryVolts);
    }

    public static final double ELEVATOR_MIN = 0;
    public static final double ELEVATOR_MAX = 1477;

    public static final double ARM_INTERFERE_MIN = (5. / 6.) * Math.PI;
    public static final double ARM_INTERFERE_MAX = (11. / 6.) * Math.PI;

    @Log.BooleanBox
    public final boolean isConflictingWithIndexer() {
        return ARM_INTERFERE_MIN < realRotaryPose && realRotaryPose < ARM_INTERFERE_MAX;
    }

    @Log.BooleanBox
    public final boolean wantsToConflictWithIndexer() {
        return wantsHandoff;
    }

    public static final synchronized Arm getInstance() { return instance == null ? instance = new Arm() : instance; }
}
