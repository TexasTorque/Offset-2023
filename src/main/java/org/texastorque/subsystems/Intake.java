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

public final class Intake extends TorqueSubsystem implements Subsystems {
    public static class IndexerPose {
        private static final double ROTARY_TOLERANCE = 0.1;

        public final double rollerVolts, rotaryPose;

        public IndexerPose(final double rollerVolts, final double rotaryPose) {
            this.rollerVolts = rollerVolts;
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

        INTAKE(new IndexerPose(6, 6.35), new IndexerPose(6, 0)),
        PRIME(new IndexerPose(0, 0)),
        UP(new IndexerPose(0, 0));

        public final IndexerPose cubePose;
        public final IndexerPose conePose;

        private State(final IndexerPose both) { this(both, both); }

        private State(final IndexerPose cubePose, final IndexerPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public IndexerPose get() { return hand.getGamePieceMode() == GamePiece.CUBE ? cubePose : conePose; }
    }

    private static volatile Intake instance;

    public static final double ROTARY_MAX_VOLTS = 12, ROLLER_MAX_VOLTS = 6;

    public static final synchronized Intake getInstance() { return instance == null ? instance = new Intake() : instance; }
    @Log.ToString
    private State activeState = State.UP;
    @Log.ToString
    private State desiredState = State.UP;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO rollers = new TorqueNEO(Ports.INTAKE_ROLLER_MOTOR);

    private final TorqueNEO rotary = new TorqueNEO(Ports.INTAKE_ROTARY_MOTOR);

    @Config
    public final PIDController rotaryPoseController = new PIDController(10, 0, 0);

    private Intake() {
        rollers.setCurrentLimit(15);
        rollers.setVoltageCompensation(12.6);
        rollers.setBreakMode(true);
        rollers.burnFlash();

        // rotary.setPositionConversionFactors();
        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();
    }

    public void setState(final State state) { this.desiredState = state; }

    public State getState() { return desiredState; }

    public boolean isState(final State state) { return getState() == state; }
    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        realRotaryPose = rotary.getPosition();

        SmartDashboard.putNumber("indexer::rotaryPose", rotary.getPosition());
        SmartDashboard.putNumber("indexer::rollersPose", rollers.getPosition());
            
        final double rollerVolts = TorqueMath.constrain(activeState.get().rollerVolts, ROLLER_MAX_VOLTS);
        SmartDashboard.putNumber("intake::requestedRollerVolts", rollerVolts);
        rollers.setVolts(rollerVolts);

        final double requestedRotaryVolts = TorqueMath.constrain(rotaryPoseController.calculate(realRotaryPose, activeState.get().rotaryPose), ROTARY_MAX_VOLTS);
        SmartDashboard.putNumber("intake::requestedRotaryVolts", requestedRotaryVolts);
        SmartDashboard.putNumber("intake::rotaryCurrent", rotary.getCurrent());
        rotary.setVolts(requestedRotaryVolts);

        if (mode.isTeleop())
            desiredState = State.UP;
    }

    @Log.BooleanBox
    public boolean isIntaking() {
        return activeState == State.INTAKE;
    }
}
