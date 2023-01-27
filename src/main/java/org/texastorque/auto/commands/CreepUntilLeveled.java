/**
 * Copyright 2023 Texas Torque.
 * 
 * This file is part of Swerve-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;

public final class CreepUntilLeveled extends TorqueCommand implements Subsystems {

    public CreepUntilLeveled() {
    }

    @Override
    protected final void init() {
        drivebase.setState(Drivebase.State.ROBOT_RELATIVE);
    }

    @Override
    protected final void continuous() {
        drivebase.inputSpeeds = new ChassisSpeeds(0, -0.75, 0);
    }

    @Override
    protected final boolean endCondition() {
        return Math.abs(TorqueNavXGyro.getInstance().getRoll()) <= 5;
    }

    @Override
    protected final void end() {
        drivebase.inputSpeeds = new ChassisSpeeds(0, 0, 0);
    }
}
