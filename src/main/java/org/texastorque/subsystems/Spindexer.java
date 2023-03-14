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
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueClick;
import org.texastorque.torquelib.motors.TorqueNEO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {

    public static enum State {
        SLOW_CW(-4), FAST_CW(-4), SLOW_CCW(2), FAST_CCW(8), OFF(0);

        public final double volts;

        private State(final double volts) {
            this.volts = volts;
        }
    }

    public static enum AutoState {
        SEARCH, FIRST_CLICK, FALSE_SWITCH, SECOND_CLICK, OFFSET, STOP;
    }

    private static volatile Spindexer instance;

    public static final synchronized Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }

    @Log.ToString
    private State state = State.OFF;
    private boolean driverWantsAutoSpindex = false, runAutoSpindex = false;
    private double secondClickPose = 0, pidVolts = 0, firstClickTimer;

    private final double TICKS_TO_ALIGN = 5;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);
    private final DigitalInput limitSwitch = new DigitalInput(0);
    private AutoState autoSpindexState = AutoState.SEARCH;
    private final PIDController turntablePID = new PIDController(1, 0, 0);

    private boolean init = false;

    private Spindexer() {
        turntable.setCurrentLimit(35);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();

    }

    public final void setState(final State state) {
        this.state = state;
    }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    @Override
    public final void initialize(final TorqueMode mode) {
    }

    public void setAutoSpindex(boolean autoSpindex) {
        this.driverWantsAutoSpindex = autoSpindex;
    }

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putBoolean("spindexer::limitSwitch", limitSwitch.get());
        SmartDashboard.putNumber("spindexer::turntablePos", turntable.getPosition());
        SmartDashboard.putString("spindexer::autoState", autoSpindexState.toString()); 
        SmartDashboard.putBoolean("spindexer::runAutoSpindex", runAutoSpindex);
        SmartDashboard.putBoolean("spindexer::driverWantsAutoSpindex", driverWantsAutoSpindex);
        SmartDashboard.putNumber("spindexer::firstClickTime", firstClickTimer);

        state = State.OFF;

        if (intake.isState(Intake.State.INTAKE) && hand.isConeMode())
            state = State.FAST_CW;

        if (driverWantsAutoSpindex) {
            if (!init) {
                autoSpindexState = AutoState.SEARCH;
                init = true;
            }

            if (autoSpindexState == AutoState.SEARCH) {
                state = State.SLOW_CW;
                if (limitSwitch.get()) {
                    autoSpindexState = AutoState.FIRST_CLICK;
                    firstClickTimer = Timer.getFPGATimestamp() / 1000;
                }

            } else if (autoSpindexState == AutoState.FIRST_CLICK) {
                state = State.SLOW_CW;
                if (!limitSwitch.get())
                    autoSpindexState = AutoState.FALSE_SWITCH;
            } else if (autoSpindexState == AutoState.FALSE_SWITCH) {
                state = State.SLOW_CW;
                if (limitSwitch.get() && Timer.getFPGATimestamp() / 1000 - firstClickTimer > 0.0002) {
                    secondClickPose = turntable.getPosition();
                    autoSpindexState = AutoState.SECOND_CLICK;
                }
            } else if (autoSpindexState == AutoState.SECOND_CLICK) {
                pidVolts = turntablePID.calculate(turntable.getPosition(), secondClickPose - TICKS_TO_ALIGN);

                if (Math.abs(turntable.getPosition()) - (Math.abs(secondClickPose) - TICKS_TO_ALIGN) < 2) {
                    autoSpindexState = AutoState.STOP;
                }
            }
        } else {
            init = false;
        }

        turntable.setVolts(autoSpindexState == AutoState.SECOND_CLICK ? pidVolts : state.volts);
    }
}