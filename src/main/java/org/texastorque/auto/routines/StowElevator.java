/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.SetArm;
import org.texastorque.auto.commands.SetHand;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.torquelib.auto.TorqueSequence;

public final class StowElevator extends TorqueSequence implements Subsystems {
    public StowElevator() {
        addBlock(new SetHand(Hand.State.CLOSE));
        addBlock(new SetArm(Arm.State.DOWN));
    }
}