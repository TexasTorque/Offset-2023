/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.*;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueBlock;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

import edu.wpi.first.wpilibj.util.Color;

public final class South extends TorqueSequence implements Subsystems {
    public South() { 
        addBlock(new TorqueExecute(() -> lights.set(Color.kBlue)));
        final var path1 = new FollowEventPath("South", true, 3, 1.5);
        path1.addEvent("switch-lights", new TorqueExecute(() -> {
            lights.set(Color.kRed);
        }));

        addBlock(path1);
    }
}