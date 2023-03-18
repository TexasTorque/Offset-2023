/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.util.TorqueUtil;

public final class FlatSide3Piece extends TorqueSequence implements Subsystems {
    private double start = 0;

    public FlatSide3Piece() {
        addBlock(new TorqueExecute(() -> start = TorqueUtil.time()));

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(hand.setGamePieceModeCommand(GamePiece.CUBE));

        addBlock(new FollowEventPath("flat-side-get-first"));

        addBlock(new TorqueWaitUntil(arm::isAtDesiredPose));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(new FollowEventPath("flat-side-get-second"));

        addBlock(new TorqueWaitUntil(arm::isAtDesiredPose));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.MID)), new TorqueContinuous(() -> {
            if (TorqueUtil.time() - start > 14.75)
                hand.setState(Hand.State.OPEN);
        }));
    }
}