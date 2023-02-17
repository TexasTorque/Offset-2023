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
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FlatSide3PieceLevel extends TorqueSequence implements Subsystems {
    public FlatSide3PieceLevel() {

        // Hack - not needed w/ april tags
        drivebase.resetPose(new Pose2d(1.8, 4.96, Rotation2d.fromRadians(Math.PI)));

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(hand.setGamePieceModeCommand(GamePiece.CUBE));

        addBlock(new FollowEventPath("flat-side-get-first", 4.5, 4.5)); 

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(new FollowEventPath("flat-side-get-second-x", 4.5, 4.5));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.MID)));

        addBlock(new FollowEventPath("flat-side-go-level-fast", 6, 8));

        addBlock(drivebase.setStateCommand(Drivebase.State.BALANCE));
    }
}