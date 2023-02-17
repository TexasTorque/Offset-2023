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
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class PickUp extends TorqueSequence implements Subsystems {
    public PickUp() {
        addBlock(new TorqueExecute(() -> arm.setState(Arm.State.HANDOFF)));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(new TorqueExecute(() -> {
            hand.setState(Hand.State.CLOSE);
            arm.setState(Arm.State.BACK);
        }));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(new TorqueExecute(() -> intake.setState(Intake.State.UP)));
    }
}