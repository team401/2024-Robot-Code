package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;

public class InterpolateDouble {
    private HashMap<Double, Double> map;
    private ArrayList<Double> sortedKeys;

    private final double minValue;
    private final double maxValue;

    private final double minKey;
    private final double maxKey;

    public InterpolateDouble(HashMap<Double, Double> map) {
        this(map, Double.MIN_VALUE, Double.MAX_VALUE);
    }

    public InterpolateDouble(HashMap<Double, Double> map, double minValue, double maxValue) {
        this.map = map;
        this.minValue = minValue;
        this.maxValue = maxValue;

        sortedKeys = new ArrayList<Double>(map.keySet());
        Collections.sort(sortedKeys);

        // Get lowest and highest keys of the HashMap
        if (sortedKeys.size() > 0) {
            minKey = sortedKeys.get(0);
            maxKey = sortedKeys.get(sortedKeys.size() - 1);
        } else {
            throw new RuntimeException("Empty HashMap passed to InterpolateDouble");
        }
    }

    /**
     * Returns the interpolated value for the given key. If the key is not in the map, it will
     * return the value for the closest key.
     *
     * @param key The key to interpolate
     * @return The interpolated value
     */
    public double getValue(double key) {
        if (map.containsKey(key)) {
            return map.get(key);
        }

        // Ensure that key is within the bounds of the HashMap
        if (key < minKey) {
            return minValue;
        } else if (key > maxKey) {
            return maxValue;
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
