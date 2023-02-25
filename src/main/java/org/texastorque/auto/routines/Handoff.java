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
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class Handoff extends TorqueSequence implements Subsystems {
    public Handoff() {
        addBlock(arm.setStateCommand(Arm.State.INDEX));
        addBlock(new TorqueExecute(() -> System.out.println("\n\nTHIS RAN\n\n")));
        addBlock(new TorqueWaitForSeconds(.3));
        addBlock(arm.setStateCommand(Arm.State.GRAB));
        addBlock(new TorqueWaitForSeconds(.3));
        addBlock(arm.setStateCommand(Arm.State.GRABBED));
        addBlock(intake.setStateCommand(Intake.State.UP));
    }
}