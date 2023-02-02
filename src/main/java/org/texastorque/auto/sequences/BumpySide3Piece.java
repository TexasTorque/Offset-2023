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
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.util.TorqueUtil;

public final class BumpySide3Piece
    extends TorqueSequence implements Subsystems {
    public BumpySide3Piece() {
        final TorqueWaitForSeconds dropInitialCone =
            new TorqueWaitForSeconds(.5);
        addBlock(dropInitialCone);

        final FollowEventPath pickUpFirstCube =
            new FollowEventPath("bumpy-side-get-first"); // ends (1.8, 1.05)
        addBlock(pickUpFirstCube);

        final TorqueWaitForSeconds dropFirstCone = new TorqueWaitForSeconds(.5);
        addBlock(dropFirstCone);

        final FollowEventPath pickUpSecondCube =
            new FollowEventPath("bumpy-side-get-second");
        addBlock(pickUpSecondCube);

        final TorqueWaitForSeconds dropSecondCube =
            new TorqueWaitForSeconds(.5);
        addBlock(dropSecondCube);
    }
}