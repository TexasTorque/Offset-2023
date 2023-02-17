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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Forks extends TorqueSubsystem implements Subsystems {
    private static volatile Forks instance;

    public static final double ROTARY_MAX_VOLTS = 6;
    

    public static final synchronized Forks getInstance() { return instance == null ? instance = new Forks() : instance; }

    private final TorqueNEO rotary = new TorqueNEO(Ports.FORKS_MOTOR);

    @Log.ToString
    private TorqueDirection direction = TorqueDirection.OFF;

    private Forks() {
        // rotary.setPositionConversionFactor(135.0);
        rotary.setCurrentLimit(40);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();
    }

    public final void setDirection(final TorqueDirection direction) {
        this.direction = direction;
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    
    @Override
    public final void update(final TorqueMode mode) {
    rotary.setVolts(ROTARY_MAX_VOLTS * direction.get());
    SmartDashboard.putNumber("forks::current", rotary.getCurrent());
    SmartDashboard.putNumber("forks::volts", ROTARY_MAX_VOLTS * direction.get());

    // direction = TorqueDirection.OFF;
    }
}
