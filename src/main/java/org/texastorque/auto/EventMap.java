package org.texastorque.auto;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Handoff;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

public final class EventMap implements Subsystems {

    public static Map<String, TorqueCommand> get() {
        final Map<String, TorqueCommand> map = new HashMap<String, TorqueCommand>();

        map.put("intake-down", new TorqueExecute(() -> {
            intake.setState(Intake.State.INTAKE);
            arm.setState(Arm.State.INDEX);
            hand.setState(Hand.State.OPEN);
        }));

        map.put("pickup", new TorqueSequenceRunner(new Handoff()));

        // Legacy
        map.put("arm-ready", new TorqueExecute(() -> arm.setState(Arm.State.TOP)));

        map.put("arm-ready-top", new TorqueExecute(() -> arm.setState(Arm.State.TOP)));
        map.put("arm-ready-mid", new TorqueExecute(() -> arm.setState(Arm.State.MID)));

        return map;
    }
}
