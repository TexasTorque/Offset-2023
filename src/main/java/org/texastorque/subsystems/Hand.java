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
        NONE, CUBE, CONE;
    }

    @Log.ToString
    public GamePiece currentGamePiece = GamePiece.NONE;

    @Log.ToString
    private GamePiece lastHeldPiece = GamePiece.NONE;

    public GamePiece getLastHeldGamePiece() {
        return lastHeldPiece;
    }

    public static enum State {
        OPEN(0), CLOSE(100);

        public final double clawPose;

        private State(final double clawPose) {
            this.clawPose = clawPose;
        }
    }

    private State state = State.OPEN;
    private State lastState = State.OPEN;
    private State requestedState = State.OPEN;
    public void setState(final State state) { this.state = state; }
    public State getState() { return requestedState; }
    public boolean isState(final State state) { return getState() == state; }

    @Log.ToString
    public double realClawPose = 0;

    private final TorqueNEO claw = new TorqueNEO(Ports.HAND_MOTOR);
    private final PIDController clawPoseController =new PIDController(0.1, 0, 0);

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
        requestedState = state;

        if (currentGamePiece != GamePiece.NONE) {
            lastHeldPiece = currentGamePiece;
        }

        // realClawPose = claw.getPosition();

        final double requestedClawVolts = clawPoseController.calculate(realClawPose, state.clawPose);
        SmartDashboard.putNumber("hand::requestedClawVolts", requestedClawVolts);
        // claw.setVolts(requestedClawVolts);

        if (lastState != state) {
            lastState = state;
            if (state == State.OPEN) {
                currentGamePiece = GamePiece.NONE;
                Input.getInstance().setDriverRumbleFor(0.5);
            }
        }
    }


    public static final synchronized Hand getInstance() {
        return instance == null ? instance = new Hand() : instance;
    }
}
