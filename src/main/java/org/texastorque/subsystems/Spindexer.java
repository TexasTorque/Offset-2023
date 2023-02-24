/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public final class Spindexer extends TorqueSubsystem implements Subsystems {
    public static enum State {
        SLOW_CW(2), FAST_CW(8), SLOW_CCW(-2), FAST_CCW(-8), OFF(0);

        public final double volts;

        private State(final double volts) {
            this.volts = volts;
        }
    }

    private static volatile Spindexer instance;

    public static final synchronized Spindexer getInstance() { return instance == null ? instance = new Spindexer() : instance; }

    private State state = State.OFF;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);

    private Spindexer() {
        turntable.setCurrentLimit(20);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();
    } 
    
    
    public final void setState(final State state) {
        this.state = state;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    
    @Override
    public final void update(final TorqueMode mode) {
        if (intake.isState(Intake.State.INTAKE))
            state = State.FAST_CW;

        turntable.setVolts(state.volts);

        state = State.OFF;
    }
}
