/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.motors.TorqueNEO.SmartMotionProfile;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.math.controller.PIDController;
import io.github.oblarg.oblog.annotations.Log;

public final class Forks extends TorqueSubsystem implements Subsystems {
    public enum State {
        UP(0), DOWN(1477), OFF(-1);

        public final double rotaryPose;

        private State(final double rotaryPose) {
            this.rotaryPose = rotaryPose;
        }

        public double getRotaryPose() {
            return rotaryPose;
        }
    }

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

    private TorqueRequestableTimeout timeout = new TorqueRequestableTimeout();

    // TODO: Tune PID!
    private final PIDController pid = new PIDController(0.00002, 0, 0);

    private State activeState = State.UP;

    private Forks() {
        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        SmartMotionProfile smp = new SmartMotionProfile(maxVelocity, 0, maxAcceleration, 1000);

        rotary.configureSmartMotion(smp);
        rotary.configurePID(TorquePID.create(.00002).build());
        rotary.burnFlash();
    }

    public final void setState(final State state) {
        activeState = state;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("current", rotary.getCurrent());
        Debug.log("velocity", rotary.getVelocity());
        Debug.log("volts", maxVelocity * direction);

        if (direction != 0) {
            timeout.set(6);
        }

        // rotary.setVolts(
        //         activeState == State.OFF ? 0 : TorqueMath.constrain(activeState.getRotaryPose(), ROTARY_MAX_VOLTS));

        // rotary.setSmartVelocity(maxVelocity * direction);

        // direction = 0;
        activeState = State.OFF;
    }

    public boolean isForksRunning() {
        return timeout.get();
    }
}
