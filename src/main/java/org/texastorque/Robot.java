/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.base.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Robot extends TorqueRobotBase implements Subsystems {
    private final ShuffleboardTab competition, pid;

    public Robot() {
        super(Input.getInstance(), AutoManager.getInstance());

        // Configure Subsystems
        addSubsystem(drivebase);
        addSubsystem(lights);
        addSubsystem(indexer);
        addSubsystem(arm);
        addSubsystem(hand);

        // Configure Competiiton dashboard
        competition = Shuffleboard.getTab("COMPETITION");
        competition.addCamera("LEFT CAMERA", drivebase.cameraLeft.getName(), "TBD").withPosition(0, 0).withSize(6, 4);
        competition.addCamera("RIGHT CAMERA", drivebase.cameraRight.getName(), "TBD").withPosition(6, 0).withSize(6, 4);
        competition.add("FIELD", drivebase.fieldMap).withPosition(0, 4).withSize(7, 4);
        competition.add("AUTO SELECTOR", AutoManager.getInstance().getAutoSelector()).withPosition(7, 4).withSize(4, 2);
        
        // PID Dashboard
        pid = Shuffleboard.getTab("PID");
        pid.add("elevator", arm.elevatorPoseController).withSize(2, 3);
        pid.add("arm", arm.rotaryPoseController).withSize(2, 3);
        pid.add("claw", hand.clawPoseController).withSize(2, 3);
    }
}
