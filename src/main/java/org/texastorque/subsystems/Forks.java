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
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.motors.TorqueNEO.SmartMotionProfile;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Forks extends TorqueSubsystem implements Subsystems {
    public enum State {
        UP(1), DOWN(-1), UP_AUTO(0), DOWN_AUTO(-10000), OFF(-1);

        public final double rotaryPose;

        private State(final double rotaryPose) {
            this.rotaryPose = rotaryPose;
        }
    }

    private static volatile Forks instance;

    public static final double ROTARY_MAX_VOLTS = 8;

    public static final synchronized Forks getInstance() {
        return instance == null ? instance = new Forks() : instance;
    }

    private final TorqueNEO rotary = new TorqueNEO(Ports.FORKS_MOTOR);

    private final double ENCODER_TO_OUTPUT_GEARING = 302.4;
    private final double MAX_VELOCITY = Math.toRadians(100) * 60 * ENCODER_TO_OUTPUT_GEARING;
    private final double MAX_ACCELERATION = 48 * 60 * ENCODER_TO_OUTPUT_GEARING;
    public double speed = 0;
    private final double LIGHTS_TIMEOUT = 6;

    private TorqueRequestableTimeout timeout = new TorqueRequestableTimeout();

    // TODO: Tune PID!
    private final PIDController pid = new PIDController(.01, 0, 0);

    private State activeState = State.UP;

    private Forks() {
        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        SmartMotionProfile smp = new SmartMotionProfile(MAX_VELOCITY, 0, MAX_ACCELERATION, 1000);

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
        // Debug.log("current", rotary.getCurrent());
        // Debug.log("velocity", rotary.getVelocity());
        // Debug.log("volts", maxVelocity * direction);

        SmartDashboard.putNumber("forks::pose", rotary.getPosition());

        if (activeState == State.OFF) {
            rotary.setVolts(0);
        } else {
            timeout.set(LIGHTS_TIMEOUT);

            if (activeState == State.UP) {
                rotary.setSmartVelocity(MAX_VELOCITY * speed);
            } else if (activeState == State.DOWN_AUTO) {
                rotary.setVolts(pid.calculate(rotary.getPosition(), activeState.rotaryPose));
            }

        }

        activeState = State.OFF;
        speed = 0;
    }

    public boolean isForksRunning() {
        return timeout.get();
    }
}