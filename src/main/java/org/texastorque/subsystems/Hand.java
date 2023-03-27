/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.sensors.TorqueCANCoder;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.PIDController;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Hand extends TorqueSubsystem implements Subsystems {
    public static enum GamePiece {
        CUBE,
        CONE;
    }

    public static enum State {
        // Smaller is bigger on the claw
        OPEN(2.9),
        SHELF(3.2),
        HALF(3.2),
        CLOSE(4);

        public final double clawSetpoint;

        private State(final double clawSetpoint) {
            this.clawSetpoint = clawSetpoint;
        }

        public double getClawSetpoint() {
            return clawSetpoint;
        }
    }

    private static volatile Hand instance;

    private static final double MAX_CLAW_VOLTS = 12;

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
    public final PIDController clawPoseController = new PIDController(12, 0, 0);

    private final TorqueNEO claw = new TorqueNEO(Ports.CLAW_MOTOR);

    private final TorqueCANCoder clawEncoder = new TorqueCANCoder(Ports.CLAW_ENCODER);

    private final TorqueRequestableTimeout retractArmTimeout = new TorqueRequestableTimeout();

    private Hand() {
        claw.setCurrentLimit(7);
        claw.setVoltageCompensation(12.6);
        claw.setBreakMode(true);
        claw.burnFlash();

        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        clawEncoder.configAllSettings(cancoderConfig);
    }

    public GamePiece getGamePieceMode() {
        return gamePieceMode;
    }

    public void setGamePieceMode(final GamePiece gamePieceMode) {
        this.gamePieceMode = gamePieceMode;
    }

    public boolean isGamePieceMode(final GamePiece gamePieceMode) {
        return this.gamePieceMode == gamePieceMode;
    }

    public boolean isCubeMode() {
        return isGamePieceMode(GamePiece.CUBE);
    }

    public boolean isConeMode() {
        return isGamePieceMode(GamePiece.CONE);
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

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    public TorqueCommand setGamePieceModeCommand(final GamePiece gamePieceMode) {
        return new TorqueExecute(() -> setGamePieceMode(gamePieceMode));
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        gamePieceMode = GamePiece.CONE;
    }

    public final boolean isClosedEnough() {
        return realClawPose >= 3.8;
    }

    @Override
    public final void update(final TorqueMode mode) {
        activeState = desiredState;
        updateFeedback();

        if (arm.isReadyToThrow())
            activeState = State.OPEN;
        if (arm.isWantingIndexClaw()) {
            activeState = isCubeMode() ? State.HALF : State.HALF;
        }
        if (arm.isState(Arm.State.SHELF) && activeState == State.OPEN) {
            activeState = State.SHELF;
        }
        if (arm.isWantGrabbyClaw())
            activeState = State.OPEN;

        double clawVolts = clawPoseController.calculate(realClawPose, activeState.clawSetpoint);
        Debug.log("requestedVolts", clawVolts);
        clawVolts = TorqueMath.constrain(clawVolts, MAX_CLAW_VOLTS);
        claw.setVolts(clawVolts);

        if (lastState != activeState) {
            if (activeState == State.OPEN)
                if (arm.isWantingScoringPose()) {
                    Input.getInstance().setDriverRumbleFor(1);
                    retractArmTimeout.set(.5);
                }
        }

        lastState = activeState;

        if (drivebase.isPathAlignDone() && activeState == State.CLOSE)
            Input.getInstance().setOperatorRumbleFor(0.5);

        if (mode.isTeleop())
            desiredState = State.CLOSE;
    }

    private void updateFeedback() {
        realClawPose = clawEncoder.getPosition();
    }
}
