/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueCurrentSpike;
import org.texastorque.torquelib.motors.TorqueNEO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Hand extends TorqueSubsystem implements Subsystems {
    public static enum GamePiece {
        CUBE,
        CONE;
    }

    public static enum State {
        OPEN(MAX_CLAW_VOLTS), // double check negation
        CLOSE(-MAX_CLAW_VOLTS),
        OFF(0);

        public final double clawVolts;

        private State(final double clawSetpoint) {
            this.clawVolts = clawSetpoint;

        }

        public double getClawVolts() {
            return clawVolts;
        }

    }

    private static volatile Hand instance;

    private static final double MAX_CLAW_VOLTS = 4;

    public static final synchronized Hand getInstance() {
        return instance == null ? instance = new Hand() : instance;
    }

    @Log.ToString
    private GamePiece gamePieceMode = GamePiece.CUBE;

    @Log.ToString
    private State activeState = State.CLOSE;
    @Log.ToString
    private State lastState = State.CLOSE;
    @Log.ToString
    private State desiredState = State.CLOSE;
    @Log.ToString
    public double realClawPose = 0;
    @Config
    public final PIDController clawPoseController = new PIDController(0.1, 0, 0);
    private TorqueCurrentSpike currentDetection = new TorqueCurrentSpike(20);
    private final DigitalInput clawSwitch;
    private final TorqueNEO claw = new TorqueNEO(Ports.HAND_MOTOR);

    private Hand() {
        claw.setCurrentLimit(20);
        claw.setVoltageCompensation(12.6);
        claw.setBreakMode(true);
        claw.burnFlash();

        clawSwitch = new DigitalInput(0);
    }

    public GamePiece getGamePieceMode() {
        return gamePieceMode;
    }

    public void setGamePieceMode(final GamePiece gamePieceMode) {
        this.gamePieceMode = gamePieceMode;
    }

    public void setState(final State state) {
        this.desiredState = state;
    }

    public State getState() {
        return desiredState;
    }

    public boolean isState(final State state) {
        return getState() == state;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putNumber("hand::requestedClawVolts", activeState.getClawVolts());
        SmartDashboard.putNumber("hand::realClawPose", claw.getPosition());
        SmartDashboard.putNumber("hand::realClawVolts", claw.getVolts());
        SmartDashboard.putString("hand::state", activeState.toString());
        activeState = desiredState;
        realClawPose = claw.getPosition();

        if (activeState == State.OFF)
            currentDetection.reset();

        if (activeState == State.OPEN) {
            if (currentDetection.calculate(claw.getCurrent()))
                activeState = State.OFF;
        } else
            currentDetection.reset();

        claw.setVolts(activeState.getClawVolts());

        if (lastState != activeState) {
            lastState = activeState;
            if (activeState == State.OPEN)
                if (arm.isAtScoringPose())
                    Input.getInstance().setDriverRumbleFor(1);
        }

        if (drivebase.isPathAlignDone() && activeState == State.CLOSE)
            Input.getInstance().setOperatorRumbleFor(0.5);

        // activeState = State.CLOSE; ?
    }
}
