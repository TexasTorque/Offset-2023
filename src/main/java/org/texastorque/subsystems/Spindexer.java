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
import org.texastorque.torquelib.motors.TorqueNEO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {

    public static enum State {
        SLOW_CW(-4), FAST_CW(-6), SLOW_CCW(4), FAST_CCW(8), OFF(0);

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
    private boolean driverWantsAutoSpindex = false;
    private double secondClickPose = 0, pidVolts = 0, firstClickTimer, grabPoseTimer;

    private final double TICKS_TO_ALIGN = 1.3;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);
    private final DigitalInput limitSwitch = new DigitalInput(0);
    private AutoState autoSpindexState = AutoState.SEARCH;
    private final PIDController turntablePID = new PIDController(2.5, 0, 0);
    private boolean initAutoSpindex = false, initGrabPose = false;

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

    public boolean isAutoSpindexing() {
        return driverWantsAutoSpindex;
    }

    @Override
    public final void update(final TorqueMode mode) {
        SmartDashboard.putBoolean("spindexer::limitSwitch", limitSwitch.get());
        SmartDashboard.putString("spindexer::autoState", autoSpindexState.toString());
        SmartDashboard.putNumber("pidDelta", Math.abs(turntable.getPosition()) - (Math.abs(secondClickPose) - TICKS_TO_ALIGN));

        if (driverWantsAutoSpindex) {
            if (!initAutoSpindex) {
                autoSpindexState = AutoState.SEARCH;
                initAutoSpindex = true;
                arm.setState(Arm.State.INDEX);
            }

            if (autoSpindexState == AutoState.SEARCH) {
                state = State.SLOW_CW;
                if (limitSwitch.get()) {
                    autoSpindexState = AutoState.FIRST_CLICK;
                    firstClickTimer = Timer.getFPGATimestamp();
                }

            } else if (autoSpindexState == AutoState.FIRST_CLICK) {
                state = State.SLOW_CW;
                if (!limitSwitch.get())
                    autoSpindexState = AutoState.FALSE_SWITCH;
            } else if (autoSpindexState == AutoState.FALSE_SWITCH) {
                // state = State.SLOW_CW;
                if (limitSwitch.get() && Timer.getFPGATimestamp() - firstClickTimer > 0.15) {
                    secondClickPose = turntable.getPosition();
                    autoSpindexState = AutoState.SECOND_CLICK;
                } else if (Timer.getFPGATimestamp() - firstClickTimer > 0.3)
                    autoSpindexState = AutoState.SEARCH;
            } else if (autoSpindexState == AutoState.SECOND_CLICK) {
                pidVolts = turntablePID.calculate(turntable.getPosition(), secondClickPose - TICKS_TO_ALIGN);

                if (Math.abs(turntable.getPosition()) - (Math.abs(secondClickPose) - TICKS_TO_ALIGN) < 2.3) {
                    autoSpindexState = AutoState.STOP;

                    if (initGrabPose) {
                        arm.setState(Arm.State.GRAB);
                        grabPoseTimer = Timer.getFPGATimestamp();
                        initGrabPose = false;
                    }

                }
            } else if (autoSpindexState == AutoState.STOP) {
                if (Timer.getFPGATimestamp() - grabPoseTimer > 0.5) {
                    arm.setState(Arm.State.GRABBED);
                 }
            }
        } else {
            initAutoSpindex = false;
            initGrabPose = true;
        }

        turntable.setVolts(autoSpindexState == AutoState.SECOND_CLICK ? pidVolts : state.volts);

        if (!driverWantsAutoSpindex || autoSpindexState == AutoState.STOP)
            state = State.OFF;
    }
}