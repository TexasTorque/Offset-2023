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
import edu.wpi.first.math.controller.PIDController;
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

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.ALIGN));

            addBlock(new TorqueWaitUntil(() -> spindexer.isConeAligned()));

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.OFF));

            addBlock(new TorqueExecute(() -> arm.resetHandoffSequence()));

            addBlock(new TorqueContinuous(() -> arm.setState(Arm.State.HANDOFF)));
        }

    }

    private static volatile Spindexer instance;

    private static double CONE_TIP_X_GOAL = 350, CONE_TIP_X_TOLERANCE = 20;

    public static final synchronized Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }


    private double volts;

    @Log.ToString
    private State desiredState = State.OFF;

    private State activeState = State.OFF;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);;

    private AutoSpindex autoSpindex;

    private final PIDController pidController = new PIDController(1, 0, 0);

    private double coneTipX;

    private Spindexer() {
        turntable.setCurrentLimit(35);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();
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

    public boolean isConeAligned() {
        // return TorqueMath.toleranced(coneTipX, CONE_TIP_X_GOAL, CONE_TIP_X_TOLERANCE);
        return 355 <= coneTipX && coneTipX <= 375;
    }

    @Override
    public final void update(final TorqueMode mode) {
        Debug.log("current", turntable.getCurrent());

        coneTipX = drivebase.getConeTipX();

        if (autoSpindex == null)
            autoSpindex = new AutoSpindex();

        if (desiredState == State.AUTO_SPINDEX) {
            autoSpindex.run();
        } else {
            activeState = desiredState;
            autoSpindex.reset();
        }

        SmartDashboard.putString("spindexer::activeState", activeState.toString());
        SmartDashboard.putNumber("spindexer::coneTipX", coneTipX);
        SmartDashboard.putBoolean("spindexer::coneTipToleranced", isConeAligned());

        volts = 0;

        if (activeState == State.ALIGN) {

            volts = coneTipX == -1477 ? 6
                    : TorqueMath.constrain(pidController.calculate(coneTipX, CONE_TIP_X_GOAL), 6);

            if (volts > 2) {
                volts = 2;
            }

            if (isConeAligned())
                volts = 0;

        } else {
            volts = activeState.volts;
        }

        desiredState = State.OFF;
        turntable.setVolts(volts);

    }

}
