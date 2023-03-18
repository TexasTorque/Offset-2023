/**
 * Copyright 2011-2023 Texas Torque.
 *
 * This file is part of TorqueLib, which is licensed under the MIT license.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import java.util.function.BooleanSupplier;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

public final class DriveUntil extends TorqueCommand implements Subsystems {

    private double velocity;
    private BooleanSupplier callback;

    public DriveUntil(final double velocity, final BooleanSupplier callback) {
        this.velocity = velocity;
        this.callback = callback;
    }

    @Override
    protected final void init() {

    }

    @Override
    protected final void continuous() {
        drivebase.inputSpeeds = new TorqueSwerveSpeeds(velocity, 0, 0);
    }

    @Override
    protected final boolean endCondition() {
        return callback.getAsBoolean();
    }

    @Override
    protected final void end() {}
}