/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.torquelib.modules.TorqueSwerveModule2022.SwervePorts;

public final class Ports {
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 11);
    public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 12);

    public static final int INDEXER_ROLLER_MOTOR = 13;
    public static final int INDEXER_ROTARY_MOTOR = 14;
    public static final int INDEXER_SPINDEXER_MOTOR = 15;
    public static final int ARM_ELEVATOR_MOTOR = 16;
    public static final int ARM_ROTARY_MOTOR = 17;
    public static final int ARM_ROTARY_ENCODER = 18;
    public static final int HAND_MOTOR = 19;
    public static final int CLIMBER_MOTOR = 20;
}
