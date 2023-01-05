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

public final class WaitForSeconds extends TorqueCommand implements Subsystems {
    private double start = 0;
    private final double time;

    public WaitForSeconds(final double time) {
        this.time = time;
    }

    @Override
    protected final void init() {
        start = TorqueUtil.time();
    }

    @Override
    protected final void continuous() {
    }

    @Override
    protected final boolean endCondition() {
        return TorqueUtil.time() - start > time;
    }

    @Override
    protected final void end() {
    }
}
