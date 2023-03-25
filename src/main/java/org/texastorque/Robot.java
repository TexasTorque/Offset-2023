/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.base.TorqueRobotBase;

public final class Robot extends TorqueRobotBase implements Subsystems {
    public Robot() {
        super(Debug.DO_LOGGING, Input.getInstance(), AutoManager.getInstance());

        // addSubsystem(drivebase);
        addSubsystem(lights);
    //    addSubsystem(intake);
        // addSubsystem(spindexer);
       addSubsystem(arm);
    //    addSubsystem(hand);
    //     addSubsystem(forks);

        Debug.initDashboard();
    }
}
