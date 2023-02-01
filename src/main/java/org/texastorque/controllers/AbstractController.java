package org.texastorque.controllers;

public abstract class AbstractController<T> {
    public abstract T calculate();
    public abstract void resetIf(final boolean reset);
}
