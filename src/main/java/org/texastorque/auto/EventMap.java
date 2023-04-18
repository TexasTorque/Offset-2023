package org.texastorque.auto;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class EventMap implements Subsystems {

    public static Map<String, TorqueCommand> get() {
        final Map<String, TorqueCommand> map = new HashMap<String, TorqueCommand>();

        map.put("intake-down", new TorqueExecute(() -> {
            intake.setState(Intake.State.AUTOINTAKE);
            hand.setState(Hand.State.HALF);
        }));

        map.put("intake-up", new TorqueExecute(() -> {
            intake.setState(Intake.State.UP);
        }));

        map.put("intake-up-2", new TorqueExecute(() -> {
            intake.setState(Intake.State.UP);
        }));

        map.put("intake-prime", new TorqueExecute(() -> {
            intake.setState(Intake.State.PRIME);
        }));

        map.put("intake-down-slow", new TorqueExecute(() -> {
            intake.setState(Intake.State.DOWN_SLOW);
        }));

        map.put("intake-down-off", new TorqueExecute(() -> {
            intake.setState(Intake.State.DOWN_OFF);
        }));

        map.put("default", new TorqueExecute(() -> {
            arm.setState(Arm.State.PRIME);
            intake.setState(Intake.State.UP);
        }));

        map.put("eject", new TorqueExecute(() -> {
            arm.setState(Arm.State.PUSH_CUBE);
            intake.setState(Intake.State.OUTAKE);
        }));

        map.put("eject-2", new TorqueExecute(() -> {
            arm.setState(Arm.State.PUSH_CUBE);
            intake.setState(Intake.State.OUTAKE);
        }));

        map.put("arm-down", new TorqueExecute(() -> {
            arm.setState(Arm.State.PRIME);
        }));

        // map.put("handoff", new TorqueSequenceRunner(new CubeHandoff()));
        map.put("handoff", new TorqueContinuous(() -> arm.setState(Arm.State.HANDOFF)));
        // map.put("pickup", new TorqueSequenceRunner(new Handoff()));
        // map.put("handoff", new TorqueSequenceRunner(new Handoff()));

        // Legacy
        map.put("arm-ready", new TorqueExecute(() -> arm.setState(Arm.State.TOP)));

        map.put("arm-ready-top", new TorqueExecute(() -> arm.setState(Arm.State.TOP)));
        map.put("arm-ready-mid", new TorqueExecute(() -> arm.setState(Arm.State.MID)));

        map.put("dump-low", intake.setStateCommand(Intake.State.OUTAKE));

        map.put("push-cube", new TorqueExecute(() -> arm.setState(Arm.State.PUSH_CUBE)));

        return map;
    }
}
