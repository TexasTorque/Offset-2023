/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences.dcmp;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class OnePieceMobilityEngage extends TorqueSequence implements Subsystems {
    public OnePieceMobilityEngage() {
        addBlock(new TorqueExecute(() -> drivebase.updateWithTags = false));
        
        drivebase.resetPose(new Pose2d(0, 0, Rotation2d.fromRadians(Math.PI))); // not needed

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(new FollowEventPath("origin-engage-out", 2.5, 3.5));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(new FollowEventPath("origin-engage-in", 2.5, 3.5));

        addBlock(drivebase.setStateCommand(Drivebase.State.BALANCE));
    }
}