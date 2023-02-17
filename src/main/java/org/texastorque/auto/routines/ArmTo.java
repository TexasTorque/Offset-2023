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
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public final class ArmTo extends TorqueSequence implements Subsystems {
    public ArmTo(final Arm.State armState) {
        addBlock(new TorqueExecute(() -> arm.setState(armState)));
     
        addBlock(new TorqueWaitUntil(() -> arm.isAtDesiredPose()));

        addBlock(new TorqueExecute(() -> hand.setState(Hand.State.OPEN)));

        addBlock(new TorqueWaitForSeconds(.31));

        addBlock(new TorqueExecute(() -> {
            arm.setState(Arm.State.BACK);
            hand.setState(Hand.State.CLOSE);
            hand.setGamePieceMode(GamePiece.CUBE);
        }));

    }
}