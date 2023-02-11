package org.texastorque.auto;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.auto.routines.RunIntake;
import org.texastorque.auto.routines.StopIntake;
import org.texastorque.auto.routines.StowElevator;
import org.texastorque.auto.routines.UpToScore;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

public final class EventMap {

    public static final Map<String, TorqueCommand> getMap() {
        return (new EventMap()).map;
    }

    private final Map<String, TorqueCommand> map = new HashMap<String, TorqueCommand>();

    private EventMap() {
        addRoutine("go-to-mid-cube", new UpToScore(Arm.State.MID, GamePiece.CUBE));
        addRoutine("go-to-mid-cone", new UpToScore(Arm.State.MID, GamePiece.CONE));
        addRoutine("go-to-top-cube", new UpToScore(Arm.State.TOP, GamePiece.CUBE));
        addRoutine("go-to-top-cone", new UpToScore(Arm.State.TOP, GamePiece.CONE));

        addRoutine("run-intake-cube", new RunIntake(GamePiece.CUBE));
        addRoutine("run-intake-cone", new RunIntake(GamePiece.CONE));

        addRoutine("stop-intake", new StopIntake());

        addRoutine("stow-elevator", new StowElevator());
    }

    private final void addRoutine(final String key, final TorqueSequence routine) {
        map.put(key, new TorqueSequenceRunner(routine));
    }


    
}
