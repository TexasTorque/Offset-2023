/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.motors.TorqueNEO.SmartMotionProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Forks extends TorqueSubsystem implements Subsystems {
    private static volatile Forks instance;

    public static final double ROTARY_MAX_VOLTS = 8;

    public static final synchronized Forks getInstance() {
        return instance == null ? instance = new Forks() : instance;
    }

    private final TorqueNEO rotary = new TorqueNEO(Ports.FORKS_MOTOR);

    @Log.ToString
    private double direction = 0;

    private final double encoderToOutputGearing = 302.4;
    private final double maxVelocity = Math.toRadians(100) * 60 * encoderToOutputGearing;
    private final double maxAcceleration = 48 * 60 * encoderToOutputGearing;

    private Forks() {
        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        SmartMotionProfile smp = new SmartMotionProfile(maxVelocity, 0, maxAcceleration, 1000);
        rotary.configureSmartMotion(smp);
        rotary.configurePID(TorquePID.create(.00002).build());
        rotary.burnFlash();
    }

    public final void setDirection(final double direction) {
        this.direction = direction;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    @Override
    public final void update(final TorqueMode mode) {
        rotary.setSmartVelocity(maxVelocity * direction);
        SmartDashboard.putNumber("forks::current", rotary.getCurrent());
        SmartDashboard.putNumber("forks::direction", direction);
        SmartDashboard.putNumber("forks::velocity", rotary.getVelocity());

        SmartDashboard.putNumber("forks::volts", maxVelocity * direction);

        direction = 0;
    }
}
