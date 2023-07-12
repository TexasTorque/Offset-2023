/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {

    public static enum State {
        AUTO_SPINDEX(0), ALIGN(0), SLOW_CW(-4), FAST_CW(-6), SLOW_CCW(4), FAST_CCW(6), OFF(0);

        public final double volts;

        private State(final double volts) {
            this.volts = volts;
        }

    }

    public static final class AutoSpindex extends TorqueSequence implements Subsystems {

        public AutoSpindex() {

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.SLOW_CCW));

            addBlock(new TorqueWaitUntil(() -> spindexer.isAligned()));

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.OFF));

            addBlock(new TorqueExecute(() -> arm.resetHandoffSequence()));

            addBlock(new TorqueContinuous(() -> arm.setState(Arm.State.HANDOFF)));
        }

    }

    private static volatile Spindexer instance;

    public static final synchronized Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }



    private double volts;


    @Log.ToString
    private State desiredState = State.OFF;

    private State activeState = State.OFF;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);;

    private AutoSpindex autoSpindex;



    private NetworkTableInstance nTInstance = NetworkTableInstance.getDefault();
    private NetworkTable spindexerTable;
    private NetworkTableEntry tipCoordsEntry;
    private Double[] tipCoords = new Double[2];


    private Spindexer() {
        turntable.setCurrentLimit(35);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();

        spindexerTable = nTInstance.getTable("spindexer");
        tipCoordsEntry = spindexerTable.getEntry("tip-coords");
        tipCoords = tipCoordsEntry.getDoubleArray(tipCoords);
    }


    public final double getEncoderPosition() {
        return turntable.getPosition();
    }

    public final void setState(final State state) {
        this.desiredState = state;
    }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    @Override
    public final void initialize(final TorqueMode mode) {}

    public boolean isAutoSpindexing() {
        return desiredState == State.AUTO_SPINDEX;
    }

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("current", turntable.getCurrent());

        if (autoSpindex == null)
            autoSpindex = new AutoSpindex();

        if (desiredState == State.AUTO_SPINDEX) {
            autoSpindex.run();
        } else {
            activeState = desiredState;
            autoSpindex.reset();
        }

        SmartDashboard.putString("spindexer::activeState", activeState.toString());

        volts = activeState.volts;

        desiredState = State.OFF;
        turntable.setVolts(volts);

    }


    public boolean isAligned() {
        return TorqueMath.toleranced(tipCoords[0], 1550, 20)
                && TorqueMath.toleranced(tipCoords[1], 760, 20);
    }
}
