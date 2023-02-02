/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Lights;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.util.Color;

public final class RunSequence extends TorqueCommand implements Subsystems {
    private final TorqueSequence sequence;

    public RunSequence(final TorqueSequence sequence) {
        this.sequence = sequence;
    }

    @Override
    protected final void init() {
    }

    @Override
    protected final void continuous() {
        sequence.run();
    }

    @Override
    protected final boolean endCondition() {
        return sequence.hasEnded();
    }

    @Override
    protected final void end() {
    }
}
