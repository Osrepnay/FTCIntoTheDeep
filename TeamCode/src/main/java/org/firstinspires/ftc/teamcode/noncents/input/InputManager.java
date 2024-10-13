package org.firstinspires.ftc.teamcode.noncents.input;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class InputManager {
    public String currentLayer = "default";
    private Map<String, List<Trigger>> triggers;

    public InputManager() {
        triggers = new HashMap<>();
        triggers.put(currentLayer, new ArrayList<>());
    }

    public void createLayer(String layerName) {
        triggers.put(layerName, new ArrayList<>());
    }

    public void addTrigger(Trigger trigger) {
        addTrigger("default", trigger);
    }

    public void addTrigger(String layer, Trigger trigger) {
        triggers.get(layer).add(trigger);
    }

    public void update() {
        List<Trigger> layerTriggers = triggers.get(currentLayer);
        for (Trigger trigger : layerTriggers) {
            trigger.update();
        }
    }
}
