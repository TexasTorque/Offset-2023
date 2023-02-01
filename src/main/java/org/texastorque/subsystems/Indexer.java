package org.texastorque.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;

public final class Indexer extends TorqueSubsystem implements Subsystems {
    private static volatile Indexer instance;

    public static enum State {
        INTAKE_CUBE(1500, 0, 0),
        INTAKE_CONE(2000, 2000, 4),
        PRIME(0, 0, 0),
        UP(0, 0, 0);

        public final double rollerVelo;
        public final double rotaryPose;
        public final double spinVolt;

        private State(final double rollerVelo, final double rotaryPose,
                      final double spinVolt) {
            this.rollerVelo = rollerVelo;
            this.rotaryPose = rotaryPose;
            this.spinVolt = spinVolt;
        }
    }

    private GamePiece lastWantedGamepiece = GamePiece.NONE;

    public GamePiece getLastWantedGamePiece() {
        return lastWantedGamepiece;
    }

    @Log.ToString(name = "State")
    public State state = State.UP;

    @Log.ToString(name = "Real Roller Velo")
    public double realRollerVelo = 0;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO rollers = new TorqueNEO(Ports.INDEXER_ROLLER_MOTOR);
    private final PIDController rollerVeloController =
        new PIDController(0.1, 0, 0);

    private final TorqueNEO rotary = new TorqueNEO(Ports.INDEXER_ROTARY_MOTOR);
    private final PIDController rotaryPoseController =
        new PIDController(0.1, 0, 0);

    private final TorqueNEO spindexer =
        new TorqueNEO(Ports.INDEXER_SPINDEXER_MOTOR);

    private Indexer() {
        // rollers.setConversionFactors();
        rollers.setCurrentLimit(20);
        rollers.setVoltageCompensation(12.6);
        rollers.setBreakMode(true);
        rollers.burnFlash();

        // rotary.setConversionFactors();
        rotary.setCurrentLimit(20);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        // spindexer.setConversionFactors();
        spindexer.setCurrentLimit(20);
        spindexer.setVoltageCompensation(12.6);
        spindexer.setBreakMode(true);
        spindexer.burnFlash();
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    @Override
    public final void update(final TorqueMode mode) {
        if (arm.wantsToConflictWithIndexer() || arm.isConflictingWithIndexer()) {
            if (wantsToConflictWithArm())
                state = State.PRIME;
        }

        if (state == State.INTAKE_CONE) {
            lastWantedGamepiece = GamePiece.CONE;
        } else if (state == State.INTAKE_CUBE) {
            lastWantedGamepiece = GamePiece.CUBE;
        }

        realRollerVelo = rollers.getVelocity();
        realRotaryPose = rotary.getPosition();

        final double requestedRollerVolts = rollerVeloController.calculate(realRollerVelo, state.rollerVelo);
        SmartDashboard.putNumber("indexer::requestedRollerVolts", requestedRollerVolts);
        rollers.setVolts(requestedRollerVolts);

        final double requestedRotaryVolts = rotaryPoseController.calculate(realRotaryPose, state.rotaryPose);
        SmartDashboard.putNumber("indexer::requestedRotaryVolts", requestedRotaryVolts);
        rotary.setVolts(requestedRotaryVolts);

        spindexer.setVolts(state.spinVolt);
    }

    public static final double INTAKE_INTERFERE_MIN = 1; // ?
    public static final double INTAKE_INTERFERE_MAX = 1; // ?

    public boolean isConflictingWithArm() {
        return INTAKE_INTERFERE_MIN < realRotaryPose && realRotaryPose < INTAKE_INTERFERE_MAX;
    }

    public boolean wantsToConflictWithArm() {
        return state == State.UP;
    }

    public boolean isIntaking() {
        return state == State.INTAKE_CUBE || state == State.INTAKE_CONE;
    }

    public static final synchronized Indexer getInstance() {
        return instance == null ? instance = new Indexer() : instance;
    }
}
