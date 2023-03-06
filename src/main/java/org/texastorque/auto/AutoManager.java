/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto;


import org.texastorque.auto.sequences.DumbBothSide1PieceMobility;
import org.texastorque.auto.sequences.DumbBumpySide1PieceEngage;
import org.texastorque.auto.sequences.DumbFlatSide1PieceEngage;
import org.texastorque.auto.sequences.DumbMiddleSide1PieceEngage;
import org.texastorque.auto.sequences.FlatSide2PieceEngage;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() { return instance == null ? instance = new AutoManager() : instance; }

    public AutoManager() { super(false); }

    @Override
    public final void init() {
        // addSequence(new BumpySide2PieceLevel());
        // addSequence(new BumpySide3Piece());
        addSequence(new FlatSide2PieceEngage());
        // addSequence(new FlatSide3Piece());
        // addSequence(new MiddleSide1PieceLevel());
        addSequence(new DumbBothSide1PieceMobility());
        addSequence(new DumbMiddleSide1PieceEngage());
        addSequence(new DumbFlatSide1PieceEngage());
        addSequence(new DumbBumpySide1PieceEngage());

    }
}