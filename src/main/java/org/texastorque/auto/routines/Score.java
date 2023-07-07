/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
import edu.wpi.first.wpilibj.Timer;

public final class Score extends TorqueSequence implements Subsystems {
    private static final double TIMEOUT = 5e3;
    private double startTime;

    public Score(final Arm.State armState) {
        this(armState, .4);
        startTime = Timer.getFPGATimestamp();
    }

    public Score(final Arm.State armState, final double waitTime) {
        addBlock(hand.setStateCommand(Hand.State.CLOSE));
        addBlock(arm.setStateCommand(armState));
        addBlock(new TorqueWaitUntil(() -> (arm.isAtState(armState)
                || Timer.getFPGATimestamp() - startTime >= TIMEOUT)));
        addBlock(new TorqueExecute(() -> hand.keepOpenInAuto = true));
        addBlock(new TorqueWaitForSeconds(waitTime));
        addBlock(hand.setStateCommand(Hand.State.CHUNGUS));
        addBlock(new TorqueWaitForSeconds(.2));
        addBlock(new TorqueExecute(() -> hand.keepOpenInAuto = false));
        addBlock(arm.setStateCommand(Arm.State.STOWED), hand.setStateCommand(Hand.State.CLOSE));

    }
}
