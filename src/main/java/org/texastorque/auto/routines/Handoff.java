package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public class Handoff extends TorqueSequence implements Subsystems {
    public Handoff() {
        goTo(Arm.State.HANDOFF_DOWN, .25);
        goTo(Arm.State.HANDOFF_GRAB, .35);
        goTo(Arm.State.HANDOFF_GRAB_BACK, .25);
    }

    private final void goTo(final Arm.State state, final double seconds) {
        addBlock(new TorqueWaitUntil(() -> {
            arm.setActiveState(state);
            return arm.isAtState(state);
        }));

        if (seconds == -1)
            return;

        addBlock(new TorqueWaitForSeconds(seconds, () -> {
            arm.setActiveState(state);
        }));
    }
}