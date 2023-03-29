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
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Intake extends TorqueSubsystem implements Subsystems {
    public static class IndexerPose {
        private static final double ROTARY_TOLERANCE = 0.1;

        public final double topRollerVolts, bottomRollerVolts, rotaryPose;

        public IndexerPose(final double topRollerVolts, final double bottomRollerVolts, final double rotaryPose) {
            this.topRollerVolts = topRollerVolts;
            this.bottomRollerVolts = bottomRollerVolts;
            this.rotaryPose = rotaryPose;
        }

        public boolean atPose(final double rotaryReal) {
            return Math.abs(rotaryReal - rotaryPose) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        INTAKE(new IndexerPose(3, 3, ROT_INTAKE), new IndexerPose(4.5, 12, ROT_INTAKE)),
        AUTOINTAKE(new IndexerPose(2, 2, ROT_INTAKE), new IndexerPose(3, 3, ROT_INTAKE)),
        OUTAKE(new IndexerPose(-4, -4, ROT_INTAKE), new IndexerPose(-8, -9, ROT_INTAKE)),
        POOP(new IndexerPose(-12, -12, -9), new IndexerPose(-8, -9, ROT_UP)),
        PRIME(new IndexerPose(0, 0, ROT_PRIME)),
        PRIME_ROLL(new IndexerPose(6, 6, ROT_PRIME), new IndexerPose(9, 12, ROT_PRIME)),
        UP(new IndexerPose(0, 0, ROT_UP)),
        UP_ROLL(new IndexerPose(-9, 0, ROT_UP));

        public final IndexerPose cubePose;
        public final IndexerPose conePose;

        private State(final IndexerPose both) {
            this(both, both);
        }

        private State(final IndexerPose cubePose, final IndexerPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public IndexerPose get() {
            return hand.isCubeMode() ? cubePose : conePose;
        }
    }

    private static final double ROT_INTAKE = -12.85;
    private static final double ROT_PRIME = -7;
    private static final double ROT_UP = 0;

    private static volatile Intake instance;

    public static final double ROTARY_MAX_VOLTS = 4, ROLLER_MAX_VOLTS = 6;

    public static final synchronized Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }

    @Log.ToString
    private State activeState = State.UP;

    @Log.ToString
    private State desiredState = State.UP;

    @Log.ToString
    private State lastState = State.UP;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO topRollers = new TorqueNEO(Ports.INTAKE_ROLLER_MOTOR_TOP);
    private final TorqueNEO bottomRollers = new TorqueNEO(Ports.INTAKE_ROLLER_MOTOR_BOTTOM);
    private final TorqueNEO rotary = new TorqueNEO(Ports.INTAKE_ROTARY_MOTOR_LEFT);

    @Config
    public final PIDController rotaryPoseController = new PIDController(2.4, 0, 0);

    private final TorqueRequestableTimeout primeRollTimeout = new TorqueRequestableTimeout();

    private Intake() {
        topRollers.setCurrentLimit(30);
        topRollers.setVoltageCompensation(12.6);
        topRollers.setBreakMode(false);
        topRollers.burnFlash();

        bottomRollers.setCurrentLimit(25);
        bottomRollers.setVoltageCompensation(12.6);
        bottomRollers.setBreakMode(false);
        bottomRollers.burnFlash();

        rotary.setCurrentLimit(33);
        rotary.setVoltageCompensation(12.6);
        rotary.addFollower(Ports.INTAKE_ROTARY_MOTOR_RIGHT, true);
        rotary.setBreakMode(true);
        rotary.burnFlash();
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

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        realRotaryPose = rotary.getPosition();

        if (desiredState == State.UP && arm.isPerformingHandoff())
            activeState = State.PRIME;

        // if (activeState == State.PRIME && (lastState == State.INTAKE || lastState == State.AUTOINTAKE))
        //     primeRollTimeout.set(1);

        // if (primeRollTimeout.get())
        //     activeState = State.PRIME_ROLL;

        Debug.log("rotaryPose", rotary.getPosition());
        Debug.log("topRollersPose", topRollers.getPosition());
        Debug.log("botRollersPose", bottomRollers.getPosition());

        double topRollerVolts = activeState.get().topRollerVolts;

        topRollers.setVolts(topRollerVolts);
        Debug.log("topRollersSpeed", topRollers.getVelocity());

        double bottomRollerVolts = -activeState.get().bottomRollerVolts;
        bottomRollers.setVolts(bottomRollerVolts);
        Debug.log("bottomRollersSpeed", bottomRollers.getVelocity());
        Debug.log("bottomRollerCurrent", bottomRollers.getCurrent());

        Debug.log("requestedRotaryPose", activeState.get().rotaryPose);
        double requestedRotaryVolts = TorqueMath.constrain(
                rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose), ROTARY_MAX_VOLTS);
        if (activeState == State.INTAKE || activeState == State.OUTAKE) {
            requestedRotaryVolts += 1;
        }
        Debug.log("requestedRotaryVolts", requestedRotaryVolts);
        Debug.log("rotaryCurrent", rotary.getCurrent());
        rotary.setVolts(requestedRotaryVolts);

        lastState = activeState;
    }

    @Log.BooleanBox
    public boolean isIntaking() {
        return activeState == State.INTAKE;
    }
}
