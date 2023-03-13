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
import org.texastorque.subsystems.Spindexer;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class Handoff extends TorqueSequence implements Subsystems {
    public Handoff() {
        addBlock(hand.setStateCommand(Hand.State.CLOSE));
        addBlock(arm.setStateCommand(Arm.State.INDEX));
        addBlock(hand.setStateCommand(Hand.State.OPEN));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(arm.setStateCommand(Arm.State.AUTOGRAB));
        // addBlock(spindexer.setStateCommand(Spindexer.State.SLOW_CW));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(hand.setStateCommand(Hand.State.CLOSE)); // will not return to default without being told to
        addBlock(new TorqueWaitForSeconds(.25));
        addBlock(intake.setStateCommand(Intake.State.UP));
        addBlock(arm.setStateCommand(Arm.State.GRABBED));
        addBlock(spindexer.setStateCommand(Spindexer.State.OFF));

        /*addBlock(arm.setStateCommand(Arm.State.INDEX));
        addBlock(new TorqueWaitForSeconds(.3));
        addBlock(arm.setStateCommand(Arm.State.GRAB));
        addBlock(spindexer.setStateCommand(Spindexer.State.SLOW_CW));
        addBlock(new TorqueWaitForSeconds(.5));
        addBlock(arm.setStateCommand(Arm.State.GRABBED));
        addBlock(spindexer.setStateCommand(Spindexer.State.OFF));
        addBlock(hand.setStateCommand(Hand.State.CLOSE)); // will not return to default without being told to
        addBlock(intake.setStateCommand(Intake.State.UP));*/
    }
}