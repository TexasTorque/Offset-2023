package org.texastorque.auto.sequences.dcmp;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

public class BumpySideThreePieceMobilityHigh extends TorqueSequence implements Subsystems {
    public BumpySideThreePieceMobilityHigh() {

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(hand.setGamePieceModeCommand(GamePiece.CUBE));

        addBlock(new FollowEventPath("bumpy-side-get-first-3"));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(intake.setStateCommand(Intake.State.UP));

        addBlock(new FollowEventPath("bumpy-side-get-second"));



        addBlock(intake.setStateCommand(Intake.State.UP));
    }
}
