package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.base.*;

public final class Robot extends TorqueRobotBase implements Subsystems {
    public Robot() {
        super(Input.getInstance(), AutoManager.getInstance());

        addSubsystem(drivebase);
        addSubsystem(lights);
        addSubsystem(indexer);
        addSubsystem(arm);
        addSubsystem(hand);
    }
}