package org.texastorque.toast.lib.pipelines;

import com.fasterxml.jackson.databind.JsonNode;

public class CenterContour extends Pipeline {

    private double tipX;

    public CenterContour() {
        super("april-tags");
    }

    public double getTipX() {
       return tipX;
    }

    @Override
    protected void init() {}

    @Override
    protected void deinit() {
    }

    @Override
    protected void update(JsonNode root) {
        root.get("contours").forEach(target -> {
            tipX = target.get("tip-x").asDouble();
        });
    }
}