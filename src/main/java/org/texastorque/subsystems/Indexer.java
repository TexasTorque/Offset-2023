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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Indexer extends TorqueSubsystem implements Subsystems {
    public static class IndexerPose {
        private static final double ROLLER_TOLERANCE = 1, ROTARY_TOLERANCE = 0.1;

        public final double rollerVelo, rotaryPose, spinVolt;

        public IndexerPose(final double rollerVelo, final double rotaryPose, final double spinVolt) {
            this.rollerVelo = rollerVelo;
            this.rotaryPose = rotaryPose;
            this.spinVolt = spinVolt;
        }

        public boolean atPose(final double elevatorReal, final double rotaryReal) {
            return Math.abs(elevatorReal - rollerVelo) < ROLLER_TOLERANCE && Math.abs(rotaryReal - rotaryPose) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        INTAKE(new IndexerPose(0, 0, 0), new IndexerPose(0, 0, 0)),
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

    public static final double INTAKE_INTERFERE_MIN = 1; // ?

    public static final double INTAKE_INTERFERE_MAX = 1; // ?

    public static final synchronized Indexer getInstance() { return instance == null ? instance = new Indexer() : instance; }
    @Log.ToString
    private State activeState = State.UP;
    @Log.ToString
    private State desiredState = State.UP;

    @Log.ToString(name = "Real Roller Velo")
    public double realRollerVelo = 0;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO rollers = new TorqueNEO(Ports.INDEXER_ROLLER_MOTOR);
    @Config
    public final PIDController rollerVeloController = new PIDController(0.1, 0, 0);

    private final TorqueNEO rotary = new TorqueNEO(Ports.INDEXER_ROTARY_MOTOR);
    @Config
    public final PIDController rotaryPoseController = new PIDController(0.1, 0, 0);

    private final TorqueNEO spindexer = new TorqueNEO(Ports.INDEXER_SPINDEXER_MOTOR);

    private Indexer() {
        // rollers.setConversionFactors();
        // rollers.setCurrentLimit(20);
        // rollers.setVoltageCompensation(12.6);
        // rollers.setBreakMode(true);
        // rollers.burnFlash();

        // rotary.setConversionFactors();
        // rotary.setCurrentLimit(20);
        // rotary.setVoltageCompensation(12.6);
        // rotary.setBreakMode(true);
        // rotary.burnFlash();

        // spindexer.setConversionFactors();
        // spindexer.setCurrentLimit(20);
        // spindexer.setVoltageCompensation(12.6);
        // spindexer.setBreakMode(true);
        // spindexer.burnFlash();
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

        // realRollerVelo = rollers.getVelocity();
        // realRotaryPose = rotary.getPosition();

        final double requestedRollerVolts = rollerVeloController.calculate(realRollerVelo, activeState.get().rollerVelo);
        SmartDashboard.putNumber("indexer::requestedRollerVolts", requestedRollerVolts);
        // rollers.setVolts(requestedRollerVolts);

        final double requestedRotaryVolts = rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose);
        SmartDashboard.putNumber("indexer::requestedRotaryVolts", requestedRotaryVolts);
        // rotary.setVolts(requestedRotaryVolts);

        SmartDashboard.putNumber("indexer::requestedSpindexerVolts", activeState.get().spinVolt);
        // spindexer.setVolts(state.spinVolt);

        activeState = State.PRIME;
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
