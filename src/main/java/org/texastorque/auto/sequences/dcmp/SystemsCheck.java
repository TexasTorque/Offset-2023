/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequences.dcmp;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Spindexer;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;

public final class SystemsCheck extends TorqueSequence implements Subsystems {
    public SystemsCheck() {
        addBlock(new FollowEventPath("systems-check", 4, 4));
        addBlock(hand.setStateCommand(Hand.State.OPEN), hand.setGamePieceModeCommand(GamePiece.CONE));
        addBlock(new TorqueExecute(() -> hand.systemsCheck = true));
        addBlock(arm.setStateCommand(Arm.State.TOP));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(arm.setStateCommand(Arm.State.MID));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(arm.setStateCommand(Arm.State.PRIME));
        addBlock(hand.setStateCommand(Hand.State.CLOSE));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(arm.setStateCommand(Arm.State.SHELF));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(intake.setStateCommand(Intake.State.INTAKE));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(intake.setStateCommand(Intake.State.PRIME));
        addBlock(new TorqueWaitForSeconds(6, () -> spindexer.setState(Spindexer.State.AUTO_SPINDEX)));
        addBlock(spindexer.setStateCommand(Spindexer.State.OFF));
        addBlock(intake.setStateCommand(Intake.State.UP));
        addBlock(arm.setStateCommand(Arm.State.MID));
        addBlock(new TorqueWaitUntil(() -> arm.isAtDesiredPose()));
        addBlock(hand.setStateCommand(Hand.State.OPEN));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(hand.setStateCommand(Hand.State.CLOSE));
        addBlock(arm.setStateCommand(Arm.State.SHELF));
        addBlock(hand.setGamePieceModeCommand(GamePiece.CUBE));
        addBlock(intake.setStateCommand(Intake.State.INTAKE));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(intake.setStateCommand(Intake.State.PRIME));
        addBlock(new TorqueWaitForSeconds(3, () -> arm.setState(Arm.State.HANDOFF)));
        addBlock(intake.setStateCommand(Intake.State.UP));
        addBlock(arm.setStateCommand(Arm.State.MID));
        addBlock(new TorqueWaitUntil(() -> arm.isAtDesiredPose()));
        addBlock(hand.setStateCommand(Hand.State.OPEN));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(hand.setStateCommand(Hand.State.CLOSE));
        addBlock(arm.setStateCommand(Arm.State.SHELF));
        addBlock(new TorqueWaitForSeconds(1));
        addBlock(new TorqueExecute(() -> forks.setDirection(-.75)));
        addBlock(new TorqueWaitForSeconds(.75));
        addBlock(new TorqueExecute(() -> forks.setDirection(.75)));
        addBlock(new TorqueWaitForSeconds(.75));
        addBlock(new TorqueExecute(() -> forks.setDirection(0)));
    }
}