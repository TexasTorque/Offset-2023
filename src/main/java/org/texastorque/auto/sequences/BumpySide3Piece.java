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

public final class BumpySide3Piece extends TorqueSequence implements Subsystems {
    private double autoStart = 0;

    public BumpySide3Piece() { 
        SmartDashboard.putNumber("ELAPSED", 0);
        addBlock(new SetIntakeMode(false));

        addBlock(new TorqueExecute(() -> autoStart = TorqueUtil.time()));

        final WaitForSeconds dropInitialCone = new WaitForSeconds(.5);
        addBlock(dropInitialCone);

        final FollowEventPath pickUpFirstCube = new FollowEventPath("bumpy-side-get-first"); // ends (1.8, 1.05)
        pickUpFirstCube.addEvent("intake-on", new SetIntakeMode(true)); 
        pickUpFirstCube.addEvent("intake-off", new SetIntakeMode(false)); 
        addBlock(pickUpFirstCube);

        final WaitForSeconds dropFirstCone = new WaitForSeconds(.5);
        addBlock(dropFirstCone);

        final FollowEventPath pickUpSecondCube = new FollowEventPath("bumpy-side-get-second");
        pickUpSecondCube.addEvent("intake-on", new SetIntakeMode(true));
        pickUpSecondCube.addEvent("intake-off", new SetIntakeMode(false)); 
        addBlock(pickUpSecondCube);

        final WaitForSeconds dropSecondCube = new WaitForSeconds(.5);
        addBlock(dropSecondCube);

        addBlock(new TorqueExecute(() -> SmartDashboard.putNumber("ELAPSED", TorqueUtil.time() - autoStart)));
    }
}