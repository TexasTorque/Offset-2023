/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class StowElevator extends TorqueSequence implements Subsystems {
    public StowElevator() {
        addBlock(new TorqueExecute(() -> arm.setState(Arm.State.BACK)));
        addBlock(new TorqueExecute(() -> hand.setState(Hand.State.CLOSE)));
    }
}