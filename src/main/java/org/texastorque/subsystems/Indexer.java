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
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Indexer extends TorqueSubsystem implements Subsystems {
    public static class IndexerPose {
        private static final double ROTARY_TOLERANCE = 0.1;

        public final double rollerVolts, rotaryPose, spinVolts;

        public IndexerPose(final double rollerVolts, final double rotaryPose, final double spinVolts) {
            this.rollerVolts = rollerVolts;
            this.rotaryPose = rotaryPose;
            this.spinVolts = spinVolts;
        }

        public boolean atPose(final double rotaryReal) {
            return  Math.abs(rotaryReal - rotaryPose) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        INTAKE(new IndexerPose(6, 0, 0), new IndexerPose(6, 0, 4)),
        PRIME(new IndexerPose(0, 0, 0)),
        UP(new IndexerPose(0, 0, 0));

        public final IndexerPose cubePose;
        public final IndexerPose conePose;

        private State(final IndexerPose both) { this(both, both); }

        private State(final IndexerPose cubePose, final IndexerPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public IndexerPose get() { return hand.getGamePieceMode() == GamePiece.CUBE ? cubePose : conePose; }
    }

    private static volatile Indexer instance;

    // TODO: Find experimentally
    public static final double INTAKE_INTERFERE_MIN = 1, INTAKE_INTERFERE_MAX = 1, ROTARY_MAX_VOLTS = 4, ROLLER_MAX_VOLTS = 4;

    public static final synchronized Indexer getInstance() { return instance == null ? instance = new Indexer() : instance; }
    @Log.ToString
    private State activeState = State.UP;
    @Log.ToString
    private State desiredState = State.UP;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO rollers = new TorqueNEO(Ports.INDEXER_ROLLER_MOTOR);

    private final TorqueNEO rotary = new TorqueNEO(Ports.INDEXER_ROTARY_MOTOR);

    @Config
    public final PIDController rotaryPoseController = new PIDController(0.1, 0, 0);

    private final TorqueNEO spindexer = new TorqueNEO(Ports.INDEXER_SPINDEXER_MOTOR);

    private Indexer() {
        rollers.setCurrentLimit(15);
        rollers.setVoltageCompensation(12.6);
        rollers.setBreakMode(true);
        rollers.burnFlash();

        // rotary.setPositionConversionFactors();
        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        spindexer.setCurrentLimit(20);
        spindexer.setVoltageCompensation(12.6);
        spindexer.setBreakMode(true);
        spindexer.burnFlash();
    }

    public void setState(final State state) { this.desiredState = state; }

    public State getState() { return desiredState; }

    public boolean isState(final State state) { return getState() == state; }
    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        // if (arm.wantsToConflictWithIndexer() || arm.isConflictingWithIndexer()) {
        //     if (wantsToConflictWithArm()) activeState = State.PRIME;
        // }

        realRotaryPose = rotary.getPosition();
            
        final double rollerVolts = TorqueMath.constrain(activeState.get().rollerVolts, ROLLER_MAX_VOLTS);
        SmartDashboard.putNumber("indexer::requestedRollerVolts", rollerVolts);
        rollers.setVolts(rollerVolts);

        final double requestedRotaryVolts = TorqueMath.constrain(rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose), ROTARY_MAX_VOLTS);
        SmartDashboard.putNumber("indexer::requestedRotaryVolts", requestedRotaryVolts);
        // rotary.setVolts(requestedRotaryVolts);

        SmartDashboard.putNumber("indexer::requestedSpindexerVolts", activeState.get().spinVolts);
        spindexer.setVolts(activeState.get().spinVolts);

        if (mode.isTeleop())
            activeState = State.UP;
    }

    @Log.BooleanBox
    public boolean isConflictingWithArm() {
        return INTAKE_INTERFERE_MIN < realRotaryPose && realRotaryPose < INTAKE_INTERFERE_MAX;
    }

    @Log.BooleanBox
    public boolean wantsToConflictWithArm() {
        return activeState == State.UP;
    }

    @Log.BooleanBox
    public boolean isIntaking() {
        return activeState == State.INTAKE;
    }
}
