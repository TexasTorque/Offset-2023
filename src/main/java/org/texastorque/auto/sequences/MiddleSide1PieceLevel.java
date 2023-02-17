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

public final class MiddleSide1PieceLevel extends TorqueSequence implements Subsystems {
    public MiddleSide1PieceLevel() {
         // Hack - not needed w/ april tags
        drivebase.resetPose(new Pose2d(2, 2.74, Rotation2d.fromRadians(Math.PI)));

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(new FollowEventPath("middle-over-and-back", 1.5, 3.5));

        addBlock(drivebase.setStateCommand(Drivebase.State.BALANCE));
    }
}