package org.texastorque;

import org.texastorque.subsystems.*;

public interface Subsystems {
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Lights lights = Lights.getInstance();
    public final Indexer indexer = Indexer.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Hand hand = Hand.getInstance();
}
