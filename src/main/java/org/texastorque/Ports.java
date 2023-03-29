/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.torquelib.swerve.TorqueSwerveModule2022.SwervePorts;

public final class Ports {

    // CAN Swerve
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 11);
    public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 12);
    // CAN Superstructure
    public static final int INTAKE_ROLLER_MOTOR_TOP = 38;
    public static final int INTAKE_ROLLER_MOTOR_BOTTOM = 39;
    public static final int INTAKE_ROTARY_MOTOR_RIGHT = 32;
    public static final int INTAKE_ROTARY_MOTOR_LEFT = 31;
    public static final int SPINDEXER_MOTOR = 15;
    public static final int ARM_ELEVATOR_MOTOR = 16;
    public static final int ARM_ROTARY_MOTOR = 17;
    public static final int ARM_ROTARY_ENCODER = 18;
    public static final int CLAW_MOTOR = 19;
    public static final int CLAW_ENCODER = 22;
    public static final int FORKS_MOTOR = 20;
    // DIO
    public static final int SPINDEXER_LIMIT_SWITCH = 4;
    public static final int SPINDEXER_US_LEFT_PING = 0;
    public static final int SPINDEXER_US_LEFT_ECHO = 1;
    public static final int SPINDEXER_US_RIGHT_PING = 3;
    public static final int SPINDEXER_US_RIGHT_ECHO = 2;
    // PWM
    public static final int LIGHTS_SUPERSTRUCTURE = 2;
    public static final int LIGHTS_UNDERGLOW = 2;

    /**
     * BRAVO:
     */

    // public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    // public static final SwervePorts FR_MOD = new SwervePorts(6, 5, 11);
    // public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    // public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 12);
    // public static final int INTAKE_ROLLER_MOTOR_TOP = 13;
    // public static final int INTAKE_ROLLER_MOTOR_BOTTOM = 21;
    // public static final int INTAKE_ROTARY_MOTOR = 14;
    // public static final int SPINDEXER_MOTOR = 15;
    // public static final int ARM_ELEVATOR_MOTOR = 16;
    // public static final int ARM_ROTARY_MOTOR = 17;
    // public static final int ARM_ROTARY_ENCODER = 18;
    // public static final int CLAW_MOTOR = 55;
    // public static final int CLAW_ENCODER = 22;
    // public static final int FORKS_MOTOR = 20; // public static final int
    // INTAKE_ROTARY_MOTOR_RIGHT = 32;
    // public static final int INTAKE_ROTARY_MOTOR_LEFT = 31;
    // public static final int CLAW_SWITCH = 0;
    // public static final int ARM_SWITCH = 1;
    // public static final int LIGHTS_SUPERSTRUCTURE = 0;
}
