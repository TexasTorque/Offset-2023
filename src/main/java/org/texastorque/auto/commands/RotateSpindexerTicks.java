/**
 * Copyright 2011-2023 Texas Torque.
 *
 * This file is part of TorqueLib, which is licensed under the MIT license.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;

public final class RotateSpindexerTicks extends TorqueCommand implements Subsystems {

    private double startPos, endPos, tolerance;
    private final PIDController pidController = new PIDController(2.5, 0, 0);

    public RotateSpindexerTicks(final double startPos, final double endPos, final double tolerance) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.tolerance = tolerance;

    }

    @Override
    protected final void init() {
    }

    @Override
    protected final void continuous() {
        spindexer.setAutoSpindexVolts(pidController.calculate(spindexer.getEncoderPosition(), endPos));
    }

    @Override
    protected final boolean endCondition() {
        return TorqueMath.toleranced(startPos, endPos, tolerance);
    }

    @Override
    protected final void end() {
    }
}