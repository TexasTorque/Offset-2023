/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.auto.sequences.cmp.Any1;
import org.texastorque.auto.sequences.cmp.Bump2;
import org.texastorque.auto.sequences.cmp.Flat3;
import org.texastorque.auto.sequences.cmp.Middle1E;
import org.texastorque.auto.sequences.cmp.Middle2E;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }

    public AutoManager() {
        super(false);
    }

    @Override
    public final void init() {
        addSequence(new Flat3());
        addSequence(new Any1());
        addSequence(new Bump2());
        addSequence(new Middle1E());
        addSequence(new Middle2E());
    }
}