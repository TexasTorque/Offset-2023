/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;

import org.texastorque.auto.sequences.*;
import org.texastorque.torquelib.auto.*;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    public AutoManager() { super(false); }

    @Override
    public final void init() {
        addSequence(new BumpySide2PieceLevel());
        addSequence(new BumpySide3Piece());
        addSequence(new FlatSide2PieceLevel());
        addSequence(new FlatSide3Piece());
        addSequence(new MiddleSide1PieceLevel());
        addSequence(new FlatSide3PieceLevel());
    }

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() { return instance == null ? instance = new AutoManager() : instance; }
}