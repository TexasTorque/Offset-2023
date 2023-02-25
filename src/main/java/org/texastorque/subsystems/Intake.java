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
            return  Math.abs(rotaryReal - rotaryPose) < ROTARY_TOLERANCE;
        }
    }

    // Reduction probably changes!
    public static enum State {
        // INTAKE(new IndexerPose(6, -6.04762 - .3095), new IndexerPose(6, -6.04762 - .3095)),
        // PRIME(new IndexerPose(0, -2.1428 - .3095)),

        INTAKE(new IndexerPose(4, 4, ROT_INTAKE), new IndexerPose(8, 9, ROT_INTAKE)),
        OUTAKE(new IndexerPose(-4, -4, ROT_INTAKE), new IndexerPose(-8, -9, ROT_INTAKE)),
        PRIME(new IndexerPose(0, 0, ROT_PRIME)),
        UP(new IndexerPose(0, 0, ROT_UP));

        public final IndexerPose cubePose;
        public final IndexerPose conePose;

        private State(final IndexerPose both) { this(both, both); }

        private State(final IndexerPose cubePose, final IndexerPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public IndexerPose get() { return hand.isCubeMode() ? cubePose : conePose; }
    }
    private static final double ROT_INTAKE = -14.5;
    private static final double ROT_PRIME = -5;
    private static final double ROT_UP = 0;

    private static volatile Intake instance;

    public static final double ROTARY_MAX_VOLTS = 7, ROLLER_MAX_VOLTS = 6;

    private static final double ROLLER_SLOWDOWN = .3;
    public static final synchronized Intake getInstance() { return instance == null ? instance = new Intake() : instance; }
    @Log.ToString
    private State activeState = State.UP;

    @Log.ToString
    private State desiredState = State.UP;

    @Log.ToString
    public double realRotaryPose = 0;
    private final TorqueNEO topRollers = new TorqueNEO(Ports.INTAKE_ROLLER_MOTOR_TOP);

    private final TorqueNEO bottomRollers = new TorqueNEO(Ports.INTAKE_ROLLER_MOTOR_BOTTOM);

    private final TorqueNEO rotary = new TorqueNEO(Ports.INTAKE_ROTARY_MOTOR);

    @Config
    public final PIDController rotaryPoseController = new PIDController(2, 0, 0);

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
        rotary.setBreakMode(true);
        rotary.burnFlash();
    }

    public void setState(final State state) { this.desiredState = state; }

    public State getState() { return desiredState; }

    public boolean isState(final State state) { return getState() == state; }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        realRotaryPose = rotary.getPosition();

        if (desiredState == State.UP && arm.isPerformingHandoff())
            activeState = State.PRIME;

        SmartDashboard.putNumber("indexer::rotaryPose", rotary.getPosition());
        SmartDashboard.putNumber("indexer::topRollersPose", topRollers.getPosition());
        SmartDashboard.putNumber("indexer::botRollersPose", bottomRollers.getPosition());

        final double rollerSlowdown = ROLLER_SLOWDOWN * TorqueMath.constrain(drivebase.inputSpeeds.getVelocityMagnitude(), 1, Drivebase.MAX_VELOCITY);

        double topRollerVolts = activeState.get().topRollerVolts;
        if (hand.isConeMode()) topRollerVolts /= rollerSlowdown;
        topRollers.setVolts(topRollerVolts);

        double bottomRollerVolts = -activeState.get().bottomRollerVolts;
        if (hand.isConeMode()) bottomRollerVolts /= rollerSlowdown;
        bottomRollers.setVolts(bottomRollerVolts);

        final double requestedRotaryVolts = TorqueMath.constrain(rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose), ROTARY_MAX_VOLTS);
        SmartDashboard.putNumber("intake::requestedRotaryVolts", requestedRotaryVolts);
        SmartDashboard.putNumber("intake::rotaryCurrent", rotary.getCurrent());
        // rotary.setVolts(requestedRotaryVolts);

        // if (mode.isTeleop())
            // desiredState = State.UP;
    }

    @Log.BooleanBox
    public boolean isIntaking() {
        return activeState == State.INTAKE;
    }
}
