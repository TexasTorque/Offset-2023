/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.ArmTo;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class FlatSide2PieceLevel extends TorqueSequence implements Subsystems {
    public FlatSide2PieceLevel() {

        // Hack - not needed w/ april tags
        drivebase.resetPose(new Pose2d(1.8, 4.96, Rotation2d.fromRadians(Math.PI)));

        addBlock(new TorqueExecute(() -> {
            hand.setState(Hand.State.CLOSE); 
            hand.setGamePieceMode(GamePiece.CONE);
        }));

        addBlock(new TorqueSequenceRunner(new ArmTo(Arm.State.TOP)));

        addBlock(new TorqueExecute(() -> hand.setGamePieceMode(GamePiece.CONE)));

        addBlock(new FollowEventPath("flat-side-get-first"));

        addBlock(new TorqueSequenceRunner(new ArmTo(Arm.State.TOP)));

        addBlock(new FollowEventPath("flat-side-go-level"));

        addBlock(new TorqueContinuous(() -> drivebase.setState(Drivebase.State.BALANCE)));
    }
}