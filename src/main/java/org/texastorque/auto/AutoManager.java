/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.auto.sequences.dcmp.OnePieceMobility;
import org.texastorque.auto.sequences.dcmp.OnePieceMobilityEngage;
import org.texastorque.auto.sequences.dcmp.SystemsCheck;
import org.texastorque.auto.sequences.dcmp.ThreePieceMobility;
import org.texastorque.auto.sequences.dcmp.ThreePieceMobilityHigh;
import org.texastorque.auto.sequences.dcmp.TwoPieceMobility;
import org.texastorque.auto.sequences.dcmp.TwoPieceMobilityEngage;
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
        addSequence(new OnePieceMobility());
        addSequence(new OnePieceMobilityEngage());
        addSequence(new TwoPieceMobility());
        addSequence(new ThreePieceMobility());
        addSequence(new TwoPieceMobilityEngage());
        addSequence(new SystemsCheck());
        addSequence(new ThreePieceMobilityHigh());
    }
}