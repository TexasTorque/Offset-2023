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
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        // cube cone
        INTAKE(new IndexerPose(8, 6, ROT_INTAKE), new IndexerPose(8, 12, ROT_INTAKE)),
        AUTOINTAKE(new IndexerPose(3, 3, ROT_INTAKE), new IndexerPose(3, 3, ROT_INTAKE)),

        OUTAKE(new IndexerPose(-4, -4, ROT_INTAKE), new IndexerPose(-8, -9, ROT_INTAKE)),
        PRIME(new IndexerPose(0, 0, ROT_PRIME)),
        UP(new IndexerPose(0, 0, ROT_UP)), 
        PRIME_ROLL(new IndexerPose(8, 0, ROT_PRIME));

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

    private static final double ROT_INTAKE = 13.5;
    private static final double ROT_PRIME = 8;

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
    public double realRotaryPose = 0;

    private final TorqueNEO topRollers = new TorqueNEO(Ports.INTAKE_ROLLER_MOTOR_TOP);
    private final TorqueNEO rotaryLeft = new TorqueNEO(Ports.INTAKE_ROTARY_MOTOR_LEFT);
    private final TorqueNEO rotaryRight = new TorqueNEO(Ports.INTAKE_ROTARY_MOTOR_RIGHT);

    @Config
    public final PIDController rotaryPoseController = new PIDController(1.5, 0, 0);


    private Intake() {
        topRollers.setCurrentLimit(40);
        topRollers.setVoltageCompensation(12.6);
        topRollers.setBreakMode(false);
        topRollers.burnFlash();

        rotaryLeft.setCurrentLimit(33);
        rotaryLeft.setVoltageCompensation(12.6);
        rotaryLeft.setBreakMode(true);
        rotaryLeft.burnFlash();

        rotaryRight.setCurrentLimit(33);
        rotaryRight.setVoltageCompensation(12.6);
        rotaryRight.setBreakMode(true);
        rotaryRight.burnFlash();

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

        realRotaryPose = rotaryLeft.getPosition();

        if (desiredState == State.UP && arm.isPerformingHandoff())
            activeState = State.PRIME;

        SmartDashboard.putNumber("indexer::rotaryPose", rotaryLeft.getPosition());
        SmartDashboard.putNumber("intake::topRollersCurrent", topRollers.getCurrent());
        SmartDashboard.putNumber("intake::rotaryCurrent", rotaryLeft.getCurrent());

        double topRollerVolts = activeState.get().topRollerVolts;
        topRollers.setVolts(-topRollerVolts);

        double requestedRotaryVolts = TorqueMath.constrain(
                rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose), ROTARY_MAX_VOLTS);
        if (activeState == State.INTAKE || activeState == State.OUTAKE) {
            requestedRotaryVolts -= .25;
        }

        rotaryLeft.setVolts(requestedRotaryVolts);
        rotaryRight.setVolts(-requestedRotaryVolts);


        if (mode.isTeleop())
            desiredState = State.UP;
    }

    @Log.BooleanBox
    public boolean isIntaking() {
        return activeState == State.INTAKE;
    }
}