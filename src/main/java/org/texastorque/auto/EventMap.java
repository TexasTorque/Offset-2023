package org.texastorque.auto;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

public final class EventMap {

    public static final Map<String, TorqueCommand> getMap() {
        return (new EventMap()).map;
    }

    private final Map<String, TorqueCommand> map = new HashMap<String, TorqueCommand>();

    private EventMap() {
    }

    private final void addRoutine(final String key, final TorqueSequence routine) {
        map.put(key, new TorqueSequenceRunner(routine));
    }
}
