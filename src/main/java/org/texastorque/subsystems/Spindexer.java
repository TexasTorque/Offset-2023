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
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;

public final class Spindexer extends TorqueSubsystem implements Subsystems {

    public static enum State {
        AUTO_SPINDEX(0), ALIGN(0), SLOW_CW(-4), FAST_CW(-6), SLOW_CCW(4), FAST_CCW(8), OFF(0);

        public final double volts;

        private State(final double volts) {
            this.volts = volts;
        }

    }

    public static final class AutoSpindex extends TorqueSequence implements Subsystems {

        public AutoSpindex() {
           
        }

    }

    private static volatile Spindexer instance;

    private final static double TICKS_TO_ALIGN = 4;

    private static final double TOLERANCE = .1;

    public static final synchronized Spindexer getInstance() {
        return instance == null ? instance = new Spindexer() : instance;
    }

    private double pidGoal = 0;

    @Log.ToString
    private State desiredState = State.OFF;;
    private State activeState = State.OFF;

    private final TorqueNEO turntable = new TorqueNEO(Ports.SPINDEXER_MOTOR);

    private final DigitalInput limitSwitch;

    private final Ultrasonic usLeft, usRight;

    private AutoSpindex autoSpindex;

    private final PIDController pidController = new PIDController(2.5, 0, 0);

    private Spindexer() {
        turntable.setCurrentLimit(35);
        turntable.setVoltageCompensation(12.6);
        turntable.setBreakMode(true);
        turntable.burnFlash();

        limitSwitch = new DigitalInput(Ports.SPINDEXER_LIMIT_SWITCH);

        usLeft = new Ultrasonic(Ports.SPINDEXER_US_LEFT_PING, Ports.SPINDEXER_US_LEFT_ECHO);
        usRight = new Ultrasonic(Ports.SPINDEXER_US_RIGHT_PING, Ports.SPINDEXER_US_RIGHT_ECHO);
        Ultrasonic.setAutomaticMode(true);

    }

    public final double getEncoderPosition() {
        return turntable.getPosition();
    }

    public final boolean isConeAligned() {
        return usLeft.getRangeInches() < 2.9 && usRight.getRangeInches() < 2.9;
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
        SmartDashboard.putNumber("spindexer::USLeft", usLeft.getRangeInches());
        SmartDashboard.putNumber("spindexer::USRight", usRight.getRangeInches());
        SmartDashboard.putBoolean("spindexer::isConeAligned", isConeAligned());

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

            volts = pidController.calculate(turntable.getPosition(), pidGoal);

            if (isEncoderAligned())
                volts = 0;
        } else {
            pidGoal = -1;
            volts = activeState.volts;
        }

        desiredState = State.OFF;
        turntable.setVolts(volts);

        // usLeft.ping();
        // usRight.ping();
    }

    public boolean isEncoderAligned() {
        return pidGoal != -1 && TorqueMath.toleranced(turntable.getPosition(), pidGoal, TOLERANCE);
    }
}