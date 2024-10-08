/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Forks;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Lights;
import org.texastorque.subsystems.Spindexer;

public interface Subsystems {
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Lights lights = Lights.getInstance();
    public final Intake intake = Intake.getInstance();
    public final Spindexer spindexer = Spindexer.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Hand hand = Hand.getInstance();
    public final Forks forks = Forks.getInstance();
}
