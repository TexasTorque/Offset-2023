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
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Forks extends TorqueSubsystem implements Subsystems {
    public static enum State {
        UP(Rotation2d.fromDegrees(0));

        public final Rotation2d rotaryPose;

        private State(final Rotation2d rotaryPose) { this.rotaryPose = rotaryPose; }
    }

    private static volatile Forks instance;

    public static final double ROTARY_MAX_VOLTS = 12;

    public static final synchronized Forks getInstance() { return instance == null ? instance = new Forks() : instance; }
    @Log.ToString
    private State activeState = State.UP;
    @Log.ToString
    private State desiredState = State.UP;

    @Log.ToString
    public Rotation2d realRotaryPose = Rotation2d.fromDegrees(0);

    private final TorqueNEO rotary = new TorqueNEO(Ports.CLIMBER_MOTOR);
    @Config
    public final PIDController rotaryPoseController = new PIDController(0.1, 0, 0);

    public final ArmFeedforward rotaryPoseFeedForward = new ArmFeedforward(0, 0, 0);

    private Forks() {
        rotary.setPositionConversionFactor(135.0); // gearing
        rotary.setCurrentLimit(60);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();
    }

    public void setState(final State state) { this.desiredState = state; }

    public State getState() { return desiredState; }

    public boolean isState(final State state) { return getState() == state; }
    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        realRotaryPose = Rotation2d.fromRotations(rotary.getPosition());

        final double rotaryFFOutput = rotaryPoseFeedForward.calculate(realRotaryPose.getRadians(), 0);

        final double rotarayPIDDOutput = -rotaryPoseController.calculate(realRotaryPose.getRadians(), activeState.rotaryPose.getRadians());
            

        final double requestedRotaryVolts = TorqueMath.constrain(rotarayPIDDOutput + rotaryFFOutput, ROTARY_MAX_VOLTS);
        SmartDashboard.putNumber("arm::requestedRotaryVolts", requestedRotaryVolts);

        rotary.setVolts(requestedRotaryVolts);
        // rotary.setVolts(0);
    }
}
