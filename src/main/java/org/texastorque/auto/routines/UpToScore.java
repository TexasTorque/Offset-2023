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
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class UpToScore extends TorqueSequence implements Subsystems {
    public UpToScore(final Arm.State armState, final GamePiece piece) {
        addBlock(new TorqueExecute(() -> hand.setGamePieceMode(piece)));
        addBlock(new SetHand(Hand.State.CLOSE));
        addBlock(new SetArm(armState));
    }
}