package org.texastorque;

import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.sensors.TorqueController;

public final class Input extends TorqueInput<TorqueController> {
    private static volatile Input instance;

    private Input() {
        driver = new TorqueController(0, 0.1);
        operator = new TorqueController(1, 0.1);
    }

    @Override
    public final void update() {
       
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
