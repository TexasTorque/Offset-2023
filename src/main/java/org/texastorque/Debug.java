/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.auto.AutoManager;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;

public final class Debug implements Subsystems {
    public static final boolean DO_LOGGING = true;

    private static final Map<String, Double> numbers = new HashMap<>();

    public static void initDashboard() {
        Shuffleboard.update();

        final ShuffleboardTab dashboard = Shuffleboard.getTab("COMPETITION");

        dashboard.add("FIELD", drivebase.fieldMap).withPosition(0, 0).withSize(7, 4);

        dashboard.add("AUTO SELECTOR", AutoManager.getInstance().getAutoSelector()).withPosition(7, 0).withSize(4, 2);

        dashboard.addBoolean("MODE", hand::isConeMode).withProperties(Map.of("Color when true", Color.kYellow.toHexString(), "Color when false", Color.kPurple.toHexString()))
                .withPosition(7, 2).withSize(4, 4);
    }

    public static void log(final String key, final double number) {
        if (DO_LOGGING) {
            if (!numbers.containsKey(key)) {
                getTab().addNumber(key, () -> numbers.get(key));
            }
            numbers.put(key, number);
        }
    }

    public static void log(final String key, final String string) {
        // if (DO_LOGGING)
        //     getTab().addString(key, () -> string);
    }

    public static void log(final String key, final Object object) {
        // if (DO_LOGGING)
        //     getTab().add(key, object);
    }

    private static ShuffleboardTab getTab() {
        // final String[] parts = TorqueUtil.getStackTraceElement(4).getClassName().split(".");
        // return Shuffleboard.getTab(parts[parts.length - 1]);
        return Shuffleboard.getTab("Debug");
    }
}
