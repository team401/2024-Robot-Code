package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class FieldFinder {
    private class FieldLocations2024 {
        public static final double STAGE_RED_NEAR_SIDE_X = 13.6; // TODO: Make these less arbitrary
        public static final double STAGE_RED_NEAR_SIDE_Y = 4.0;
        public static final double STAGE_RED_LEFT_SIDE_X = 10.8;
        public static final double STAGE_RED_LEFT_SIDE_Y = 2.2;
        public static final double STAGE_RED_RIGHT_SIDE_X = 10.805;
        public static final double STAGE_RED_RIGHT_SIDE_Y = 5.8;

        public static final double STAGE_BLUE_NEAR_SIDE_X = 3.0;
        public static final double STAGE_BLUE_NEAR_SIDE_Y = 4.0;
        public static final double STAGE_BLUE_LEFT_SIDE_X = 5.605;
        public static final double STAGE_BLUE_LEFT_SIDE_Y = 5.8;
        public static final double STAGE_BLUE_RIGHT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_RIGHT_SIDE_Y = 2.2;

        public static final double WING_RED_END_X = 10.8;

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

        double yIntercept1 = (b / m - b1 / m1) / (1 / m - 1 / m1);
        double yIntercept2 = (b / m - b2 / m2) / (1 / m - 1 / m2);
        double yIntercept3 = (b / m - b3 / m3) / (1 / m - 1 / m3);

        boolean distance1IsHit =
                MathUtil.isNear(x, xIntercept1, Math.abs(dx))
                        && MathUtil.isNear(y, yIntercept1, Math.abs(dy));
        boolean distance2IsHit =
                MathUtil.isNear(x, xIntercept2, Math.abs(dx))
                        && MathUtil.isNear(y, yIntercept2, Math.abs(dy));
        boolean distance3IsHit =
                MathUtil.isNear(x, xIntercept3, Math.abs(dx))
                        && MathUtil.isNear(y, yIntercept3, Math.abs(dy));

        return inTheTriangle(x, y, x1, y1, x2, y2, x3, y3)
                || (distance1IsHit
                        && isOnCorrectSide(x, xIntercept1, dx)
                        && isOnCorrectSide(y, yIntercept1, dy)
                        && isBetween(xIntercept1, x1, x2)
                        && isBetween(yIntercept1, y1, y2))
                || (distance2IsHit
                        && isOnCorrectSide(x, xIntercept2, dx)
                        && isOnCorrectSide(y, yIntercept2, dy)
                        && isBetween(xIntercept2, x2, x3)
                        && isBetween(yIntercept2, y2, y3))
                || (distance3IsHit
                        && isOnCorrectSide(x, xIntercept3, dx)
                        && isOnCorrectSide(y, yIntercept3, dy)
                        && isBetween(xIntercept3, x3, y1)
                        && isBetween(yIntercept3, y3, y1));
    }

    private static boolean isBetween(double value, double edgeOne, double edgeTwo) {
        return (value > edgeOne && value < edgeTwo) || (value < edgeOne && value > edgeTwo);
    }

    private static boolean isOnCorrectSide(double value, double intercept, double delta) {
        return (intercept > value && delta > 0) || (intercept < value && delta < 0);
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
