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

public final class MiddleSide1PieceLevel extends TorqueSequence implements Subsystems {
    public MiddleSide1PieceLevel() {
        final TorqueWaitForSeconds dropInitialCone = new TorqueWaitForSeconds(.5);
        addBlock(dropInitialCone);

        final FollowEventPath goOverChargeStation = new FollowEventPath("middle-over-and-back", 1.5, 3.5);
        addBlock(goOverChargeStation);

        addBlock(new TorqueContinuous(() -> drivebase.setState(Drivebase.State.BALANCE)));
    }
}