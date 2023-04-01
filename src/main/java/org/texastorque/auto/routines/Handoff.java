/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueSequence;

public final class Handoff extends TorqueSequence implements Subsystems {
    public Handoff() {
        // addBlock(hand.setStateCommand(Hand.State.CLOSE));
        // addBlock(arm.setStateCommand(Arm.State.INDEX));
        // addBlock(hand.setStateCommand(Hand.State.OPEN));
        // addBlock(new TorqueWaitForSeconds(1));
        // addBlock(arm.setStateCommand(Arm.State.GRAB));
        // addBlock(new TorqueWaitForSeconds(.4));
        // addBlock(intake.setStateCommand(Intake.State.PRIME));
        // addBlock(hand.setStateCommand(Hand.State.CLOSE)); // will not return to default without being told to
        // addBlock(new TorqueWaitForSeconds(.25));
        // addBlock(intake.setStateCommand(Intake.State.UP));
        // addBlock(arm.setStateCommand(Arm.State.INDEX));
        // addBlock(spindexer.setStateCommand(Spindexer.State.OFF));
    }
}