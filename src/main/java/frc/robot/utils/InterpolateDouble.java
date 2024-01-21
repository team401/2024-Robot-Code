package frc.robot.utils;

import java.util.HashMap;

public class InterpolateDouble {
    private HashMap<Double, Double> map;

    public InterpolateDouble(HashMap<Double, Double> map) {
        this.map = map;
    }

    public double getValue(double key) {
        if (map.containsKey(key)) {
            return map.get(key);
        }

        double lowerKey = 0;
        double upperKey = 0;
        for (double k : map.keySet()) {
            if (k < key && k > lowerKey) {
                lowerKey = k;
            } else if (k > key && k < upperKey) {
                upperKey = k;
            }
        }

        double lowerValue = map.get(lowerKey);
        double upperValue = map.get(upperKey);

        return lowerValue + (key - lowerKey) * (upperValue - lowerValue) / (upperKey - lowerKey);
    }
}
