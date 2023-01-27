package org.texastorque.auto;

import org.texastorque.auto.sequences.*;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    @Override
    public final void init() {
        addSequence(new BumpySide2Piece()); // only exists because it costs nothing
        addSequence(new BumpySide2PieceLevel());
        addSequence(new BumpySide3Piece());
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