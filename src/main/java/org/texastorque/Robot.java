/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import java.util.Map;

import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.base.TorqueRobotBase;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;

public final class Robot extends TorqueRobotBase implements Subsystems {
    public Robot() {
        super(Input.getInstance(), AutoManager.getInstance());

        Shuffleboard.update();

        // Configure Subsystems
        addSubsystem(drivebase);
        addSubsystem(lights);
        addSubsystem(intake);
        addSubsystem(spindexer);
        addSubsystem(arm);
        addSubsystem(hand);
        // addSubsystem(forks);

        final ShuffleboardTab dashboard = Shuffleboard.getTab("COMPETITION");

        // dashboard.addCamera("LEFT CAMERA", drivebase.cameraLeft.getName(), "http://10.14.77.79:1182/stream.mjpg")
        //         .withPosition(0, 0)
        //         .withSize(6, 4)
        //         .withProperties(Map.of("Show crosshair", false, "Show controls", false));

        // dashboard.addCamera("RIGHT CAMERA", drivebase.cameraRight.getName(), "http://10.14.77.105:1182/stream.mjpg")
        //         .withPosition(6, 0)
        //         .withSize(6, 4)
        //         .withProperties(Map.of("Show crosshair", false, "Show controls", false));

        var w = BuiltInWidgets.kBooleanBox;



        dashboard.add("FIELD", drivebase.fieldMap).withPosition(0, 0).withSize(7, 4);

        dashboard.add("AUTO SELECTOR", AutoManager.getInstance().getAutoSelector()).withPosition(7, 0).withSize(4, 2);

        dashboard.addBoolean("MODE", hand::isConeMode).withProperties(Map.of("Color on true", Color.kYellow.toHexString(), "Color on false", Color.kPurple.toHexString()))
                .withPosition(7, 2).withSize(2, 2);
    }
}
