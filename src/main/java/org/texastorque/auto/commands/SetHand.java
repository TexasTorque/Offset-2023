package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Hand;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.util.TorqueUtil;

public final class SetHand extends TorqueCommand implements Subsystems {
    private final Hand.State state;
    private double start = 0;  

    public SetHand(final Hand.State state) { this.state = state; }

    @Override
    protected final void init() {
        hand.setState(state);
        start = TorqueUtil.time();
    }

    @Override
    protected final void continuous() {}

    @Override
    protected final boolean endCondition() {
        return TorqueUtil.time() - start > 0.1;
    }

    @Override
    protected final void end() {}
}

