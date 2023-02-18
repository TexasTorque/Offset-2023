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
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class Handoff extends TorqueSequence implements Subsystems {
    // This must be reworked!
    public Handoff() {
        addBlock(arm.setStateCommand(Arm.State.HANDOFF));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(hand.setStateCommand(Hand.State.CLOSE), arm.setStateCommand(Arm.State.BACK));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(intake.setStateCommand(Intake.State.UP));
    }
}