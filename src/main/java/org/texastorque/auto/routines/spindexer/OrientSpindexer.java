package org.texastorque.auto.routines.spindexer;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Spindexer;
import org.texastorque.torquelib.auto.TorqueSequence;

public final class OrientSpindexer extends TorqueSequence implements Subsystems {
    // If the limit switch is inside of the cone when the sequence starts, then it
    // will be aligned after one click, so it waits for 1/4 of a second to allow the
    // US to recalibrate. If the limit switch is outside of the cone, then it will
    // be aligned after one click and an offset (Then wait for 1/4 of a second to
    // allow the US to recalibrate)
    public OrientSpindexer() {
        // addBlock(new TorqueExecute(() ->
        // spindexer.setAutoSpindexVolts(Spindexer.State.FAST_CW.getVolts())));
        addBlock(spindexer.setStateCommand(Spindexer.State.FAST_CW));
        // addBlock(new TorqueWaitUntil(() -> spindexer.limitSwitch.get()));
        // // addBlock(new TorqueWaitForSeconds(0.25));
        // // addBlock(new TorqueExecute(() ->
        // // spindexer.setAutoSpindexVolts(Spindexer.State.FAST_CW.getVolts())));
        // // addBlock(new RotateSpindexerTicks(turntable.getPosition(),
        // // turntable.getPosition() + TICKS_TO_ALIGN,
        // // TOLERANCE));
        // addBlock(new TorqueExecute(() ->
        // spindexer.setAutoSpindexVolts(Spindexer.State.OFF.getVolts())));
        // addBlock(new TorqueWaitForSeconds(0.25));
    }
}