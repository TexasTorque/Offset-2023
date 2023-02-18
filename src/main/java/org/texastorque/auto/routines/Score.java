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
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public final class Score extends TorqueSequence implements Subsystems {
    public Score(final Arm.State armState) {
        addBlock(arm.setStateCommand(armState));
        addBlock(new TorqueWaitUntil(() -> arm.isAtDesiredPose()));
        addBlock(hand.setStateCommand(Hand.State.OPEN));
        addBlock(new TorqueWaitForSeconds(.31));
        addBlock(arm.setStateCommand(Arm.State.BACK), hand.setStateCommand(Hand.State.CLOSE));

    }
}