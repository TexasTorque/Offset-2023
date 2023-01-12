package org.texastorque.auto;

import org.texastorque.auto.sequences.*;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void init() {
        addSequence("Test Drivebase", new DriveTest());
        addSequence("South", new South());
    }

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }
}