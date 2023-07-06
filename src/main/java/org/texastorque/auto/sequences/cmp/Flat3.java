/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences.cmp;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

public final class Flat3 extends TorqueSequence implements Subsystems {
    public Flat3() {
        // addBlock(new TorqueExecute(() -> drivebase.updateWithTags = false));

        addBlock(hand.setStateCommand(Hand.State.CLOSE),
                hand.setGamePieceModeCommand(GamePiece.CONE));

        // Score cone high:
        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        // Dumb cube low:
        // addBlock(new TorqueWaitForSeconds(1, () ->  intake.setState(Intake.OUTTAKE)));
        // addBlocK(new TorqueExecute(() -> intake.setState(Intake.State.UP)));

        addBlock(hand.setGamePieceModeCommand(GamePiece.CUBE));

        // // Score cube mid:
        // addBlock(new TorqueSequenceRunner(new Score(Arm.State.MID)));

        addBlock(new FollowEventPath("flat-first-3"));

        addBlock(intake.setStateCommand(Intake.State.UP));
        addBlock(arm.setStateCommand(Arm.State.PRIME));

        addBlock(new FollowEventPath("flat-second-3"));

        // addBlock(intake.setStateCommand(Intake.State.UP));
    }
}
