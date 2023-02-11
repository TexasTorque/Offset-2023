package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.torquelib.auto.TorqueCommand;

public final class SetArm extends TorqueCommand implements Subsystems {
    private final Arm.State state;

    public SetArm(final Arm.State state) { this.state = state; }

    @Override
    protected final void init() {
        arm.setState(state);
    }

    @Override
    protected final void continuous() {}

    @Override
    protected final boolean endCondition() {
        return arm.isAtDesiredPose();
    }

    @Override
    protected final void end() {}
}

