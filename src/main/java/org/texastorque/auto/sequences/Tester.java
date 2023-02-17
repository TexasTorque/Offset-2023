/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Tester extends TorqueSequence implements Subsystems {
    public Tester() {


        addBlock(new TorqueExecute(() -> {
            hand.setState(Hand.State.CLOSE); 
            hand.setGamePieceMode(GamePiece.CONE);
        }));
        
        addBlock(new TorqueSequenceRunner(new ArmGoTo(Arm.State.TOP)));

        final FollowEventPath path = new FollowEventPath("test", 2, 2); // ends (1.8, 1.05)

        path.addEvent("intake-down", new TorqueExecute(() -> {
            intake.setState(Intake.State.INTAKE);
            arm.setState(Arm.State.DOWN);
            hand.setState(Hand.State.OPEN);
        }));

        path.addEvent("pickup", new TorqueSequenceRunner(new Pickup()));

        path.addEvent("arm-ready", new TorqueExecute(() -> arm.setState(Arm.State.TOP)));

        addBlock(path);

        addBlock(new TorqueSequenceRunner(new ArmGoTo(Arm.State.TOP)));
    }
}