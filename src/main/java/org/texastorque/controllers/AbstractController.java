/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.controllers;

public abstract class AbstractController<T> {
    public abstract T calculate();
    public abstract void resetIf(final boolean reset);
}
