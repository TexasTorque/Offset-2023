/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.Robot;
import org.texastorque.Subsystems;
import org.texastorque.auto.commands.*;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.util.TorqueUtil;

public final class FlatSide3PieceLevel
    extends TorqueSequence implements Subsystems {
    public FlatSide3PieceLevel() {
        final TorqueWaitForSeconds dropInitialCone =
            new TorqueWaitForSeconds(.5);
        addBlock(dropInitialCone);

        final FollowEventPath pickUpFirstCube = new FollowEventPath(
            "flat-side-get-first", 4.5, 4.5); // ends (1.8, 1.05)
        addBlock(pickUpFirstCube);

        final TorqueWaitForSeconds dropFirstCone = new TorqueWaitForSeconds(.5);
        addBlock(dropFirstCone);

        final FollowEventPath pickUpSecondCube =
            new FollowEventPath("flat-side-get-second-x", 4.5, 4.5);
        addBlock(pickUpSecondCube);

        final TorqueWaitForSeconds dropSecondCube =
            new TorqueWaitForSeconds(.5);
        addBlock(dropSecondCube);

        final FollowEventPath goToLevel =
            new FollowEventPath("flat-side-go-level-fast", 6, 8);
        addBlock(goToLevel);

        addBlock(new TorqueContinuous(
            () -> drivebase.setState(Drivebase.State.BALANCE)));
    }
}