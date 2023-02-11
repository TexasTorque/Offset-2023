/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Indexer;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class StopIntake extends TorqueSequence implements Subsystems {
    public StopIntake() {
        addBlock(new TorqueExecute(() -> indexer.setState(Indexer.State.PRIME)));
    }
}