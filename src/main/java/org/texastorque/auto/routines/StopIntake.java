/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class StopIntake extends TorqueSequence implements Subsystems {
    public StopIntake() {
        addBlock(new TorqueExecute(() -> intake.setState(Intake.State.PRIME)));
        addBlock(new TorqueExecute(() -> {
            if (arm.isState(Arm.State.HANDOFF)) 
                arm.setState(Arm.State.DOWN);
        }));
    }
}