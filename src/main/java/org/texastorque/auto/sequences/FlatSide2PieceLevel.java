/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.*;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class FlatSide2PieceLevel extends TorqueSequence implements Subsystems {
    public FlatSide2PieceLevel() {
        final TorqueWaitForSeconds dropInitialCone = new TorqueWaitForSeconds(.5);
        addBlock(dropInitialCone);

        final FollowEventPath pickUpFirstCube = new FollowEventPath("flat-side-get-first"); // ends (1.8, 1.05)
        addBlock(pickUpFirstCube);

        final TorqueWaitForSeconds dropFirstCone = new TorqueWaitForSeconds(.5);
        addBlock(dropFirstCone);

        final FollowEventPath goToLevel = new FollowEventPath("flat-side-go-level");
        addBlock(goToLevel);

        addBlock(new TorqueContinuous(() -> drivebase.setState(Drivebase.State.BALANCE)));
    }
}