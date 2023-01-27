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
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Mid extends TorqueSequence implements Subsystems {
    private double autoStart = 0;

    public Mid() { 
        SmartDashboard.putNumber("ELAPSED", 0);

        addBlock(new TorqueExecute(() -> autoStart = TorqueUtil.time()));

        final WaitForSeconds wait1 = new WaitForSeconds(.5);
        addBlock(wait1);

        addBlock(new SetIntakeMode(false));

        final FollowEventPath mid1 = new FollowEventPath("mid-1", 1.5, 2);
        addBlock(mid1);

        addBlock(new TorqueExecute(() -> SmartDashboard.putNumber("ELAPSED", TorqueUtil.time() - autoStart)));
    }
}