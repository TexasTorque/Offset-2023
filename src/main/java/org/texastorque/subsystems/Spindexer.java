/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.auto.commands.RotateSpindexerTicks;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueRunSequenceWhile;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {

    public static enum State {
        AUTO_SPINDEX(autoVolts), SLOW_CW(-4), FAST_CW(-8), SLOW_CCW(4), FAST_CCW(8), OFF(0);

        public final double volts;

        private State(final double volts) {
            this.volts = volts;
        }

        private double getVolts() {
            return volts;
        }
    }

    public final class AutoSpindex extends TorqueSequence implements Subsystems {

        public AutoSpindex() {
            addBlock(new TorqueWaitForSeconds(0.25));
            addBlock(new TorqueRunSequenceWhile(new OrientSpindexer(), () -> spindexer.isConeAligned()));
            // addBlock(new TorqueSequenceRunner(new Handoff()));
        }

    }

    public final class OrientSpindexer extends TorqueSequence implements Subsystems {
        // If the limit switch is inside of the cone when the sequence starts, then it
        // will be aligned after one click, so it waits for 1/4 of a second to allow the
        // US to recalibrate. If the limit switch is outside of the cone, then it will
        // be aligned after one click and an offset (Then wait for 1/4 of a second to
        // allow the US to recalibrate)
        public OrientSpindexer() {
            addBlock(new TorqueExecute(() -> spindexer.setAutoSpindexVolts(Spindexer.State.FAST_CW.getVolts())));
            addBlock(new TorqueWaitUntil(() -> spindexer.limitSwitch.get()));
            addBlock(new TorqueWaitForSeconds(0.25));
            addBlock(new TorqueExecute(() -> spindexer.setAutoSpindexVolts(Spindexer.State.FAST_CW.getVolts())));
            addBlock(new RotateSpindexerTicks(turntable.getPosition(), turntable.getPosition() + TICKS_TO_ALIGN,
                    TOLERANCE));
            addBlock(new TorqueExecute(() -> spindexer.setAutoSpindexVolts(Spindexer.State.OFF.getVolts())));
            addBlock(new TorqueWaitForSeconds(0.25));
        }
    }

    public final class Handoff extends TorqueSequence implements Subsystems {
        public Handoff() {
            addBlock(Arm.getInstance().setStateCommand(Arm.State.GRAB));
            addBlock(new TorqueWaitForSeconds(0.5));
            addBlock(Arm.getInstance().setStateCommand(Arm.State.GRABBED));
        }
    }

    private static volatile Spindexer instance;

    private static double autoVolts;

    public static final synchronized Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }

    @Log.ToString
    private State desiredState = State.OFF;

    private State activeState = State.OFF;
    private final TorqueNEO turntable =  new TorqueNEO(Ports.SPINDEXER_MOTOR);
    ;
    private final DigitalInput limitSwitch;

    private final Ultrasonic usLeft, usRight;

    private final double TICKS_TO_ALIGN = .2, TOLERANCE = .1;

    private final AutoSpindex autoSpindex = new AutoSpindex();

    private Spindexer() {
        turntable.setCurrentLimit(35);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();

        limitSwitch = new DigitalInput(Ports.SPINDEXER_LIMIT_SWITCH);

        usLeft = new Ultrasonic(Ports.SPINDEXER_US_LEFT_PING, Ports.SPINDEXER_US_LEFT_ECHO);
        usRight = new Ultrasonic(Ports.SPINDEXER_US_RIGHT_PING, Ports.SPINDEXER_US_RIGHT_ECHO);
        // Ultrasonic.setAutomaticMode(true);
    }

    public final double getEncoderPosition() {
        return turntable.getPosition();
    }

    public final void setAutoSpindexVolts(final double volts) {
        autoVolts = volts;
    }

    public final boolean isConeAligned() {
        return usLeft.getRangeInches() < 3.3 && usRight.getRangeInches() < 3.3;
    }

    public final void setActiveState(final State state) {
        this.activeState = state;
    }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setActiveState(state));
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
        SmartDashboard.putNumber("spindexer::USLeft", usLeft.getRangeInches());
        SmartDashboard.putNumber("spindexer::USRight", usRight.getRangeInches());
        SmartDashboard.putBoolean("spindexer::isConeAligned", isConeAligned());
        SmartDashboard.putString("spindexer::activeState", activeState.toString());

        activeState = desiredState;

        if (desiredState == State.AUTO_SPINDEX)
            autoSpindex.run();
        else
            autoSpindex.reset();

        turntable.setVolts(activeState.volts);

        desiredState = State.OFF;

        usLeft.ping();
        usRight.ping();
    }
}