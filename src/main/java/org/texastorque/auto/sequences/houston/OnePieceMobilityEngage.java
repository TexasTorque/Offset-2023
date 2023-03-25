/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences.houston;

import org.texastorque.Debug;
import org.texastorque.Subsystems;
import org.texastorque.auto.commands.DriveUntil;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;

public final class OnePieceMobilityEngage extends TorqueSequence implements Subsystems {
    public OnePieceMobilityEngage() {
        // addBlock(new TorqueExecute(() -> drivebase.updateWithTags = false));
        
        // drivebase.resetPose(new Pose2d(2, 2, Rotation2d.fromRadians(Math.PI))); // not needed


        addBlock(new TorqueExecute(() -> Debug.log("state", "not started")));

        addBlock(hand.setStateCommand(Hand.State.CLOSE), hand.setGamePieceModeCommand(GamePiece.CONE));

        addBlock(new TorqueSequenceRunner(new Score(Arm.State.TOP)));

        addBlock(new TorqueExecute(() -> Debug.log("state", "driving too")));

        addBlock(new DriveUntil(2, () -> TorqueNavXGyro.getInstance().getPitch() > 10));

        addBlock(new TorqueExecute(() -> Debug.log("state", "driving on")));

        addBlock(new DriveUntil(2, () -> TorqueNavXGyro.getInstance().getPitch() < -10
        ));

        addBlock(new TorqueExecute(() -> Debug.log("state", "driving off")));

        addBlock(new DriveUntil(2, () -> TorqueNavXGyro.getInstance().getPitch() > -1 && TorqueNavXGyro.getInstance().getPitch() < 1));

        addBlock(new TorqueExecute(() -> Debug.log("state", "doing path")));

        addBlock(new FollowEventPath("off-station", 3, 2));

        // addBlock(new DriveUntil(-.3, () -> TorqueNavXGyro.getInstance().getPitch() > -5));

        addBlock(drivebase.setStateCommand(Drivebase.State.BALANCE));
    }
}