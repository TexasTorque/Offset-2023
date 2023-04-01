/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {

    public static enum State {
        AUTO_SPINDEX(0), ALIGN(0), SLOW_CW(-4), FAST_CW(-4), SLOW_CCW(4), FAST_CCW(4), OFF(0);

        public final double volts;

        private State(final double volts) {
            this.volts = volts;
        }

    }

    public static final class AutoSpindex extends TorqueSequence implements Subsystems {

        public AutoSpindex() {
            // addBlock(new TorqueWaitForSeconds(.25));

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.FAST_CCW));

            addBlock(new TorqueWaitUntil(() -> spindexer.limitSwitch.get()));

            // addBlock(new TorqueWaitForSeconds(1));

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.ALIGN));

            addBlock(new TorqueWaitUntil(() -> spindexer.isEncoderAligned()));

            addBlock(new TorqueExecute(() -> spindexer.activeState = State.OFF));
        }

    }

    private static volatile Spindexer instance;

    // private final static double TICKS_TO_ALIGN = 8;
    private final static double TICKS_TO_ALIGN = 17;

    private static final double TOLERANCE = .5;

    public static final synchronized Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }

    private double pidGoal = -1;

    @Log.ToString
    private State desiredState = State.OFF;;
    private State activeState = State.OFF;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);

    private final DigitalInput limitSwitch;

    private AutoSpindex autoSpindex;

    private final PIDController pidController = new PIDController(1, 0, 0);

    private Spindexer() {
        turntable.setCurrentLimit(35);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();

        limitSwitch = new DigitalInput(4);
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
    public final void initialize(final TorqueMode mode) {
    }

    public boolean isAutoSpindexing() {
        return desiredState == State.AUTO_SPINDEX;
    }

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putBoolean("spindexer::limitSwitch", limitSwitch.get());

        if (autoSpindex == null)
            autoSpindex = new AutoSpindex();

        if (desiredState == State.AUTO_SPINDEX) {
            autoSpindex.run();
        } else {
            activeState = desiredState;
            // autoSpindex = new AutoSpindex();
            autoSpindex.reset();
        }

        SmartDashboard.putString("spindexer::activeState", activeState.toString());

        double volts = 0;

        if (activeState == State.ALIGN) {
            if (pidGoal == -1) {
                pidGoal = turntable.getPosition() - TICKS_TO_ALIGN;
            }

            volts = TorqueMath.constrain(pidController.calculate(turntable.getPosition(), pidGoal), 4);

            if (isEncoderAligned())
                volts = 0;
        } else {
            pidGoal = -1;
            volts = activeState.volts;
        }

        desiredState = State.OFF;
        turntable.setVolts(volts);

    }

    public boolean isEncoderAligned() {
        return pidGoal != -1 && TorqueMath.toleranced(turntable.getPosition(), pidGoal, TOLERANCE);
    }
}