/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueDirection;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {
    private static volatile Spindexer instance;

    public static final double SPINDEXER_MAX_VOLTS = 4;

    public static final synchronized Spindexer getInstance() { return instance == null ? instance = new Spindexer() : instance; }

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);

    @Log.ToString
    private TorqueDirection direction = TorqueDirection.OFF;
    private Spindexer() {
        turntable.setCurrentLimit(20);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();
    } 
    
    
    public final void setDirection(final TorqueDirection direction) {
        this.direction = direction;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    
    @Override
    public final void update(final TorqueMode mode) {
        turntable.setVolts(SPINDEXER_MAX_VOLTS * direction.get());

        direction = TorqueDirection.OFF;
    }
}
