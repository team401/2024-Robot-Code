package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class FieldFinder {
    private class FieldLocations2024 {
        public static final double STAGE_RED_NEAR_SIDE_X = 13.6; // TODO: Make these less arbitrary
        public static final double STAGE_RED_NEAR_SIDE_Y = 4.0;
        public static final double STAGE_RED_LEFT_SIDE_X = 10.8;
        public static final double STAGE_RED_LEFT_SIDE_Y = 2.2;
        public static final double STAGE_RED_RIGHT_SIDE_X = 10.8;
        public static final double STAGE_RED_RIGHT_SIDE_Y = 5.8;

        public static final double STAGE_BLUE_NEAR_SIDE_X = 3.0;
        public static final double STAGE_BLUE_NEAR_SIDE_Y = 4.0;
        public static final double STAGE_BLUE_LEFT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_LEFT_SIDE_Y = 5.8;
        public static final double STAGE_BLUE_RIGHT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_RIGHT_SIDE_Y = 2.2;

        public static final double WING_RED_END_X = 8.8;

        public static final double WING_BLUE_END_X = 5.8;
    }

    public enum FieldLocations {
        RED_STAGE,
        BLUE_STAGE,
        RED_WING,
        BLUE_WING,
        MIDDLE
    }

    /**
     * This method is used to determine where the robot is on the field
     *
     * @param x The x position of the robot
     * @param y The y position of the robot
     * @return The location on the field
     */
    public static FieldLocations whereAmI(double x, double y) {

        if (inTheTriangle(
                x,
                y,
                FieldLocations2024.STAGE_RED_NEAR_SIDE_X,
                FieldLocations2024.STAGE_RED_NEAR_SIDE_Y,
                FieldLocations2024.STAGE_RED_LEFT_SIDE_X,
                FieldLocations2024.STAGE_RED_LEFT_SIDE_Y,
                FieldLocations2024.STAGE_RED_RIGHT_SIDE_X,
                FieldLocations2024.STAGE_RED_RIGHT_SIDE_Y)) {
            return FieldLocations.RED_STAGE;
        } else if (inTheTriangle(
                x,
                y,
                FieldLocations2024.STAGE_BLUE_NEAR_SIDE_X,
                FieldLocations2024.STAGE_BLUE_NEAR_SIDE_Y,
                FieldLocations2024.STAGE_BLUE_LEFT_SIDE_X,
                FieldLocations2024.STAGE_BLUE_LEFT_SIDE_Y,
                FieldLocations2024.STAGE_BLUE_RIGHT_SIDE_X,
                FieldLocations2024.STAGE_BLUE_RIGHT_SIDE_Y)) {
            return FieldLocations.BLUE_STAGE;
        } else if (x > FieldLocations2024.WING_RED_END_X) {
            return FieldLocations.RED_WING;
        } else if (x < FieldLocations2024.WING_BLUE_END_X) {
            return FieldLocations.BLUE_WING;
        } else {
            return FieldLocations.MIDDLE;
        }
    }

    /**
     * This method is used to determine if the robot will hit a location on the field
     *
     * @param x The x position of the robot
     * @param y The y position of the robot
     * @param dx The change in, or delta, x of the robot (not to be confused with x velocity)
     * @param dy The change in, or delta, y of the robot (not to be confused with y velocity)
     * @param location The location on the field to check
     * @return {@code true} if the robot will hit the location
     */
    public static boolean willIHitThis(
            double x, double y, double dx, double dy, FieldLocations location) {
        switch (location) {
            case RED_STAGE:
                return willIHitTriangle(
                        x,
                        y,
                        dx,
                        dy,
                        FieldLocations2024.STAGE_RED_NEAR_SIDE_X,
                        FieldLocations2024.STAGE_RED_NEAR_SIDE_Y,
                        FieldLocations2024.STAGE_RED_LEFT_SIDE_X,
                        FieldLocations2024.STAGE_RED_LEFT_SIDE_Y,
                        FieldLocations2024.STAGE_RED_RIGHT_SIDE_X,
                        FieldLocations2024.STAGE_RED_RIGHT_SIDE_Y);
            case BLUE_STAGE:
                return willIHitTriangle(
                        x,
                        y,
                        dx,
                        dy,
                        FieldLocations2024.STAGE_BLUE_NEAR_SIDE_X,
                        FieldLocations2024.STAGE_BLUE_NEAR_SIDE_Y,
                        FieldLocations2024.STAGE_BLUE_LEFT_SIDE_X,
                        FieldLocations2024.STAGE_BLUE_LEFT_SIDE_Y,
                        FieldLocations2024.STAGE_BLUE_RIGHT_SIDE_X,
                        FieldLocations2024.STAGE_BLUE_RIGHT_SIDE_Y);
            case RED_WING:
                return x > FieldLocations2024.WING_RED_END_X
                        || x + dx > FieldLocations2024.WING_RED_END_X;
            case BLUE_WING:
                return x < FieldLocations2024.WING_BLUE_END_X
                        || x + dx < FieldLocations2024.WING_BLUE_END_X;
            case MIDDLE:
                return (x < FieldLocations2024.WING_RED_END_X
                                || x + dx < FieldLocations2024.WING_RED_END_X)
                        && (x > FieldLocations2024.WING_BLUE_END_X
                                || x + dx > FieldLocations2024.WING_BLUE_END_X);
            default:
                return false;
        }
    }

    private static boolean willIHitTriangle(
            double x,
            double y,
            double dx,
            double dy,
            double x1,
            double y1,
            double x2,
            double y2,
            double x3,
            double y3) {
        double m = dy / dx;

        double m1 = (y2 - y1) / (x2 - x1);
        double m2 = (y3 - y2) / (x3 - x2);
        double m3 = (y1 - y3) / (x1 - x3);

        double b = y - m * x;

        double b1 = y1 - m1 * x1;
        double b2 = y2 - m2 * x2;
        double b3 = y3 - m3 * x3;

        double xIntercept1 = (b1 - b) / (m - m1);
        double xIntercept2 = (b2 - b) / (m - m2);
        double xIntercept3 = (b3 - b) / (m - m3);

        // y = mx + b
        // y - b = mx
        // (y - b) / m = x
        // (y - b) / m = (y - b1) / m1

        double yIntercept1 = m * (xIntercept1 - y3) + y1;
        double yIntercept2 = m * (xIntercept2 - y1) + y2;
        double yIntercept3 = m * (xIntercept3 - y2) + y3;

        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        double distance1 = Math.sqrt(Math.pow(xIntercept1 - x, 2) + Math.pow(yIntercept1 - y, 2));
        double distance2 = Math.sqrt(Math.pow(xIntercept2 - x, 2) + Math.pow(yIntercept2 - y, 2));
        double distance3 = Math.sqrt(Math.pow(xIntercept3 - x, 2) + Math.pow(yIntercept3 - y, 2));

        return inTheTriangle(x, y, x1, y1, x2, y2, x3, y3)
                || distance1 < distance
                        && (((xIntercept1 > x && dx > 0) || (xIntercept1 < x && dx < 0))
                                && ((yIntercept1 > y && dy > 0) || (yIntercept1 < y && dy < 0)))
                        && ((xIntercept1 > x1 && xIntercept1 < x2)
                                || (xIntercept1 < x1 && xIntercept1 > x2))
                        && ((yIntercept1 > y1 && yIntercept1 < y2)
                                || (yIntercept1 < y1 && yIntercept1 > y2))
                || distance2 < distance
                        && (((xIntercept2 > x && dx > 0) || (xIntercept2 < x && dx < 0))
                                && ((yIntercept2 > y && dy > 0) || (yIntercept2 < y && dy < 0)))
                        && ((xIntercept2 > x2 && xIntercept2 < x3)
                                || (xIntercept2 < x2 && xIntercept2 > x3))
                        && ((yIntercept2 > y2 && yIntercept2 < y3)
                                || (yIntercept2 < y2 && yIntercept2 > y3))
                || distance3 < distance
                        && (((xIntercept3 > x && dx > 0) || (xIntercept3 < x && dx < 0))
                                && ((yIntercept3 > y && dy > 0) || (yIntercept3 < y && dy < 0)))
                        && ((xIntercept3 > x3 && xIntercept3 < x1)
                                || (xIntercept3 < x3 && xIntercept3 > x1))
                        && ((yIntercept3 > y3 && yIntercept3 < y1)
                                || (yIntercept3 < y3 && yIntercept3 > y1));
    }

    private static boolean inTheTriangle(
            double x, double y, double x1, double y1, double x2, double y2, double x3, double y3) {
        double area = areaOfTriangle(x1, y1, x2, y2, x3, y3);

        double a1 = areaOfTriangle(x, y, x2, y2, x3, y3);
        double a2 = areaOfTriangle(x1, y1, x, y, x3, y3);
        double a3 = areaOfTriangle(x1, y1, x2, y2, x, y);

        return MathUtil.isNear(area, a1 + a2 + a3, 0.05);
    }

    private static double areaOfTriangle(
            double x1, double y1, double x2, double y2, double x3, double y3) {
        return 0.5 * Math.abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    }
}
