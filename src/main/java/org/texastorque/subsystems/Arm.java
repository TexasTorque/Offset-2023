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

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class Arm extends TorqueSubsystem implements Subsystems {
    private static volatile Arm instance;

    public static class ArmPose {
        public final double elevatorPose;
        public final double rotaryPose;

        public ArmPose(final double elevatorPose, final double rotaryPose) {
            this.elevatorPose = elevatorPose;
            this.rotaryPose = rotaryPose;
        }
    }

    public static enum State {
        HANDOFF(
            new ArmPose(0, 0),
            new ArmPose(0, 0)
        ), // Position to grab from indexer (contacts indexer)
        DOWN(
            new ArmPose(0, 0),
            new ArmPose(0, 0)
        ), // Position to grab from ground (contacts ground)
        SHELF(
            new ArmPose(0, 0),
            new ArmPose(0, 0)
        ), // Position to grab from shelf (contacts shelf)
        MID(
            new ArmPose(0, 0),
            new ArmPose(0, 0)
        ), // Position to grab from human player (contacts human player)
        TOP(
            new ArmPose(0, 0),
            new ArmPose(0, 0)
        ); // Position to intake (contacts intake)
       
        public final ArmPose cubePose;
        public final ArmPose conePose;

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public ArmPose get() {
            return hand.getLastHeldGamePiece() == GamePiece.CUBE ? cubePose : conePose;
        }
    }

    public boolean isAtScoringPose() {
        return state == State.MID || state == State.TOP;
    }

    @Log.ToString(name = "State")
    public State state = State.HANDOFF;

    @Log.ToString
    public double realElevatorPose = 0;

    @Log.ToString
    public double realRotaryPose = 0;

    private final TorqueNEO elevator = new TorqueNEO(Ports.ARM_ELEVATOR_MOTOR);
    private final PIDController elevatorPoseController =
        new PIDController(0.1, 0, 0);

    private final TorqueNEO rotary = new TorqueNEO(Ports.ARM_ROTARY_MOTOR);
    private final PIDController rotaryPoseController =
        new PIDController(0.1, 0, 0);

    private final CANCoder rotaryEncoder = new CANCoder(Ports.ARM_ROTARY_ENCODER);
    private static final double ARM_ROTARY_ENCODER_OFFSET= 0;

    private Arm() {
        // elevator.setConversionFactors();
        elevator.setCurrentLimit(20);
        elevator.setVoltageCompensation(12.6);
        elevator.setBreakMode(true);
        elevator.burnFlash();

        // rotary.setConversionFactors();
        rotary.setCurrentLimit(20);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    private boolean wantsHandoff = false;

    @Override
    public final void update(final TorqueMode mode) {
        wantsHandoff = state == State.HANDOFF;
        if (wantsHandoff && indexer.isConflictingWithArm())
            state = State.DOWN;

        realElevatorPose = elevator.getVelocity();
        realRotaryPose = rotary.getPosition() - ARM_ROTARY_ENCODER_OFFSET;

        final double requestedElevatorVolts = elevatorPoseController.calculate(realElevatorPose, state.get().elevatorPose);
        SmartDashboard.putNumber("arm::requestedElevatorVolts", requestedElevatorVolts);
        elevator.setVolts(requestedElevatorVolts);

        final double requestedRotaryVolts = rotaryPoseController.calculate(realRotaryPose, state.get().rotaryPose);
        SmartDashboard.putNumber("arm::requestedRotaryVolts", requestedRotaryVolts);
        rotary.setVolts(requestedRotaryVolts);
    }

    public static final double ARM_INTERFERE_MIN = (5. / 6.) * Math.PI;
    public static final double ARM_INTERFERE_MAX = (11. / 6.) * Math.PI;

    public final boolean isConflictingWithIndexer() {
        return ARM_INTERFERE_MIN < realRotaryPose && realRotaryPose < ARM_INTERFERE_MAX;
    }

    public final boolean wantsToConflictWithIndexer() {
        return wantsHandoff; 
    }

    public static final synchronized Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }
}
