/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;

public final class Empty extends TorqueSubsystem {
    private static volatile Empty instance;

    private Empty() {}

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {}

    public static final synchronized Empty getInstance() {
        return instance == null ? instance = new Empty() : instance;
    }
}
