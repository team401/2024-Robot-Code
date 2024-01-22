package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

public class InterpolateDouble {
    private HashMap<Double, Double> map;
    private ArrayList<Double> sortedKeys;

    public InterpolateDouble(HashMap<Double, Double> map) {
        this.map = map;

        sortedKeys = new ArrayList<Double>(map.keySet());
        Collections.sort(sortedKeys);
    }

    public double getValue(double key) {
        if (map.containsKey(key)) {
            return map.get(key);
        }

        double lowerKey = 0;
        double upperKey = 0;
        for (double k : sortedKeys) {
            if (k < key) {
                lowerKey = k;
            } else {
                upperKey = k;
                break;
            }
        }

        double lowerValue = map.get(lowerKey);
        double upperValue = map.get(upperKey);

        double t = (key - lowerKey) / (upperKey - lowerKey);
        return lowerValue * (1.0 - t) + t * upperValue;
    }
}
