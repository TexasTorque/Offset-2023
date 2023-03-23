/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences.houston;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public final class TwoPieceMobility extends TorqueSequence implements Subsystems {
    public TwoPieceMobility() {

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(hand.setGamePieceModeCommand(GamePiece.CUBE));

        addBlock(new FollowEventPath("flat-side-get-first"));

        addBlock(new TorqueWaitUntil(arm::isAtDesiredPose));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        // addBlock(new FollowEventPath("flat-side-get-ready"));


        // addBlock(arm.setStateCommand(Arm.State.STOWED));
        // addBlock(new TorqueWaitForSeconds(2));
        // addBlock(new TorqueSequenceRunner(new Handoff()));
    }
}