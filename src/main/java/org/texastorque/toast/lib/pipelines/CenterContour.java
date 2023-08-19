package org.texastorque.toast.lib.pipelines;

import com.fasterxml.jackson.databind.JsonNode;

public class CenterContour extends Pipeline {

    private double tipX;

    public CenterContour() {
        super("auto-spindexer");
    }

    public double getTipX() {
        return tipX;
    }

    @Override
    protected void init() {}

    @Override
    protected void deinit() {}

    @Override
    protected void update(JsonNode root) {
        tipX = root.get("tip-x").asDouble();
    }
}
