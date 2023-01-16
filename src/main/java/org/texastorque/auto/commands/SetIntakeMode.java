/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.util.Color;

public final class SetIntakeMode extends TorqueCommand implements Subsystems {
    private final boolean on;

    public SetIntakeMode(final boolean on) {
        this.on = on;
    }

    @Override
    protected final void init() {
        lights.set(on ? Color.kGreen : Color.kRed, false);
    }

    @Override
    protected final void continuous() {
    }

    @Override
    protected final boolean endCondition() {
        return true;
    }

    @Override
    protected final void end() {
    }
}
