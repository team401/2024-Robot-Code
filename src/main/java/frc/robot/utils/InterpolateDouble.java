package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

public class InterpolateDouble {
    private HashMap<Double, Double> map;
    private ArrayList<Double> sortedKeys;

    private final double minValue;
    private final double maxValue;

    public InterpolateDouble(HashMap<Double, Double> map) {
        this(map, Double.MIN_VALUE, Double.MAX_VALUE);
    }

    public InterpolateDouble(HashMap<Double, Double> map, double minValue, double maxValue) {
        this.map = map;
        this.minValue = minValue;
        this.maxValue = maxValue;

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
        double result = lowerValue * (1.0 - t) + t * upperValue;
        if (result < minValue) {
            return minValue;
        } else if (result > maxValue) {
            return maxValue;
        } else {
            return result;
        }
    }
}
