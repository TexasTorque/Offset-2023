/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public final class Hand extends TorqueSubsystem implements Subsystems {
    private static volatile Hand instance;

    public static enum GamePiece {
        NONE,
        CUBE,
        CONE;
    }

    @Log.ToString public GamePiece currentGamePiece = GamePiece.NONE;

    @Log.ToString private GamePiece lastHeldPiece = GamePiece.NONE;

    public GamePiece getLastHeldGamePiece() { return lastHeldPiece; }

    public static enum State {
        OPEN(0),
        CLOSE(100);

        public final double clawPose;

        private State(final double clawPose) { this.clawPose = clawPose; }
    }

    @Log.ToString
    private State activeState = State.CLOSE;
    @Log.ToString
    private State lastState = State.CLOSE;
    @Log.ToString
    private State desiredState = State.CLOSE;
    public void setState(final State state) { this.desiredState = state; }
    public State getState() { return desiredState; }
    public boolean isState(final State state) { return getState() == state; }

    @Log.ToString public double realClawPose = 0;

    private final TorqueNEO claw = new TorqueNEO(Ports.HAND_MOTOR);
    public final PIDController clawPoseController =
        new PIDController(0.1, 0, 0);

    private Hand() {
        // rollers.setConversionFactors();
        claw.setCurrentLimit(20);
        claw.setVoltageCompensation(12.6);
        claw.setBreakMode(true);
        claw.burnFlash();
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;

        if (currentGamePiece != GamePiece.NONE) {
            lastHeldPiece = currentGamePiece;
        }

        // realClawPose = claw.getPosition();

        final double requestedClawVolts =
            clawPoseController.calculate(realClawPose, activeState.clawPose);
        SmartDashboard.putNumber("hand::requestedClawVolts",
                                 requestedClawVolts);
        // claw.setVolts(requestedClawVolts);

        if (lastState != activeState) {
            lastState = activeState;
            if (activeState == State.OPEN) {
                currentGamePiece = GamePiece.NONE;
                if (arm.isAtScoringPose())
                    Input.getInstance().setDriverRumbleFor(1);
            } else {
                currentGamePiece = indexer.getLastWantedGamePiece();
            }
        }


        if (drivebase.isPathAlignDone() && activeState == State.CLOSE)
            Input.getInstance().setOperatorRumbleFor(0.5);

        activeState = State.CLOSE;
    }

    public static final synchronized Hand getInstance() {
        return instance == null ? instance = new Hand() : instance;
    }
}
