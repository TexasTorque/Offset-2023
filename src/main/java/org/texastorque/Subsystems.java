/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.*;

public interface Subsystems {
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Lights lights = Lights.getInstance();
    public final Indexer indexer = Indexer.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Hand hand = Hand.getInstance();
}
