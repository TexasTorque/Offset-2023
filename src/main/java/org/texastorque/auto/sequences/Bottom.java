/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Robot;
import org.texastorque.Subsystems;
import org.texastorque.auto.commands.*;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class Bottom extends TorqueSequence implements Subsystems {
    public Bottom() { 
        addBlock(new SetIntakeMode(false));

        final FollowEventPath bottom1 = new FollowEventPath("bottom-1_c"); // ends (1.8, 1.05)
        bottom1.addEvent("intake-on", new SetIntakeMode(true)); 
        bottom1.addEvent("intake-off", new SetIntakeMode(false)); 
        addBlock(bottom1);

        final WaitForSeconds wait1 = new WaitForSeconds(.5);
        addBlock(wait1);

        final FollowEventPath bottom2 = new FollowEventPath("bottom-2_a");
        addBlock(bottom2);
    }
}