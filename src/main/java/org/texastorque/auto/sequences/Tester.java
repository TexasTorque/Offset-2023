/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class Tester extends TorqueSequence implements Subsystems {
    public Tester() {
        final TorqueWaitForSeconds dropInitialCone = new TorqueWaitForSeconds(.5);
        addBlock(dropInitialCone);

        final FollowEventPath test = new FollowEventPath("test", 2, 2); // ends (1.8, 1.05)
        addBlock(test);

        addBlock(new TorqueContinuous(() -> drivebase.setState(Drivebase.State.BALANCE)));
    }
}