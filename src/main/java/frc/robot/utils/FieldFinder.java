package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import java.awt.geom.Line2D;

public class FieldFinder {
    private class FieldLocations2024 {
        public static final double STAGE_RED_NEAR_SIDE_X = 13.6; // TODO: Make these less arbitrary
        public static final double STAGE_RED_NEAR_SIDE_Y = 4.0;
        public static final double STAGE_RED_LEFT_SIDE_X = 10.8;
        public static final double STAGE_RED_LEFT_SIDE_Y = 2.2;
        public static final double STAGE_RED_RIGHT_SIDE_X = 10.8;
        public static final double STAGE_RED_RIGHT_SIDE_Y = 5.8;

        // TODO: Measure actual podium rectangle dimensions
        public static final double PODIUM_RED_NEAR_SIDE_LEFT_X = STAGE_RED_NEAR_SIDE_X + 0.1;
        public static final double PODIUM_RED_NEAR_SIDE_LEFT_Y = STAGE_RED_NEAR_SIDE_Y - 0.1;
        public static final double PODIUM_RED_NEAR_SIDE_RIGHT_X = STAGE_RED_NEAR_SIDE_X + 0.1;
        public static final double PODIUM_RED_NEAR_SIDE_RIGHT_Y = STAGE_RED_NEAR_SIDE_Y + 0.1;
        public static final double PODIUM_RED_FAR_SIDE_RIGHT_X = STAGE_RED_NEAR_SIDE_X;
        public static final double PODIUM_RED_FAR_SIDE_RIGHT_Y = STAGE_RED_NEAR_SIDE_Y + 0.1;
        public static final double PODIUM_RED_FAR_SIDE_LEFT_X = STAGE_RED_NEAR_SIDE_X;
        public static final double PODIUM_RED_FAR_SIDE_LEFT_Y = STAGE_RED_NEAR_SIDE_Y - 0.1;

        public static final double STAGE_BLUE_NEAR_SIDE_X = 3.0;
        public static final double STAGE_BLUE_NEAR_SIDE_Y = 4.0;
        public static final double STAGE_BLUE_LEFT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_LEFT_SIDE_Y = 5.8;
        public static final double STAGE_BLUE_RIGHT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_RIGHT_SIDE_Y = 2.2;

        // TODO: Measure actual podium rectangle dimensions
        public static final double PODIUM_BLUE_NEAR_SIDE_LEFT_X = STAGE_BLUE_NEAR_SIDE_X - 0.1;
        public static final double PODIUM_BLUE_NEAR_SIDE_LEFT_Y = STAGE_BLUE_NEAR_SIDE_Y + 0.1;
        public static final double PODIUM_BLUE_NEAR_SIDE_RIGHT_X = STAGE_BLUE_NEAR_SIDE_X - 0.1;
        public static final double PODIUM_BLUE_NEAR_SIDE_RIGHT_Y = STAGE_BLUE_NEAR_SIDE_Y - 0.1;
        public static final double PODIUM_BLUE_FAR_SIDE_RIGHT_X = STAGE_BLUE_NEAR_SIDE_X;
        public static final double PODIUM_BLUE_FAR_SIDE_RIGHT_Y = STAGE_BLUE_NEAR_SIDE_Y - 0.1;
        public static final double PODIUM_BLUE_FAR_SIDE_LEFT_X = STAGE_BLUE_NEAR_SIDE_X;
        public static final double PODIUM_BLUE_FAR_SIDE_LEFT_Y = STAGE_BLUE_NEAR_SIDE_Y + 0.1;

        public static final double WING_RED_END_X = 10.8;

        public static final double WING_BLUE_END_X = 5.8;
    }

    public enum FieldLocations {
        RED_STAGE,
        RED_PODIUM,
        BLUE_STAGE,
        BLUE_PODIUM,
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
            case RED_PODIUM:
                return willIHitQuad(
                        x,
                        y,
                        dx,
                        dy,
                        FieldLocations2024.PODIUM_RED_NEAR_SIDE_LEFT_X,
                        FieldLocations2024.PODIUM_RED_NEAR_SIDE_LEFT_Y,
                        FieldLocations2024.PODIUM_RED_NEAR_SIDE_RIGHT_X,
                        FieldLocations2024.PODIUM_RED_NEAR_SIDE_RIGHT_Y,
                        FieldLocations2024.PODIUM_RED_FAR_SIDE_RIGHT_X,
                        FieldLocations2024.PODIUM_RED_FAR_SIDE_RIGHT_Y,
                        FieldLocations2024.PODIUM_RED_FAR_SIDE_LEFT_X,
                        FieldLocations2024.PODIUM_RED_FAR_SIDE_LEFT_Y);
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
            case BLUE_PODIUM:
                return willIHitQuad(
                        x,
                        y,
                        dx,
                        dy,
                        FieldLocations2024.PODIUM_BLUE_NEAR_SIDE_LEFT_X,
                        FieldLocations2024.PODIUM_BLUE_NEAR_SIDE_LEFT_Y,
                        FieldLocations2024.PODIUM_BLUE_NEAR_SIDE_RIGHT_X,
                        FieldLocations2024.PODIUM_BLUE_NEAR_SIDE_RIGHT_Y,
                        FieldLocations2024.PODIUM_BLUE_FAR_SIDE_RIGHT_X,
                        FieldLocations2024.PODIUM_BLUE_FAR_SIDE_RIGHT_Y,
                        FieldLocations2024.PODIUM_BLUE_FAR_SIDE_LEFT_X,
                        FieldLocations2024.PODIUM_BLUE_FAR_SIDE_LEFT_Y);
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
        return inTheTriangle(x, y, x1, y1, x2, y2, x3, y3)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x1, y1, x2, y2)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x2, y2, x3, y3)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x3, y3, x1, y1);
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

    private static boolean willIHitQuad(
            double x,
            double y,
            double dx,
            double dy,
            double x1,
            double y1,
            double x2,
            double y2,
            double x3,
            double y3,
            double x4,
            double y4) {
        return inTheQuad(x, y, x1, y1, x2, y2, x3, y3, x4, y4)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x1, y1, x2, y2)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x2, y2, x3, y3)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x3, y3, x4, y4)
                || Line2D.linesIntersect(x, y, x + dx, y + dy, x4, y4, x1, y1);
    }

    private static boolean inTheQuad(
            double x,
            double y,
            double x1,
            double y1,
            double x2,
            double y2,
            double x3,
            double y3,
            double x4,
            double y4) {
        double area = areaOfQuad(x1, y1, x2, y2, x3, y3, x4, y4);

        double a1 = areaOfQuad(x, y, x2, y2, x3, y3, x4, y4);
        double a2 = areaOfQuad(x1, y1, x, y, x3, y3, x4, y4);
        double a3 = areaOfQuad(x1, y1, x2, y2, x, y, x4, y4);
        double a4 = areaOfQuad(x1, y1, x2, y2, x3, y3, x, y);

        return MathUtil.isNear(area, a1 + a2 + a3 + a4, 0.05);
    }

    private static double areaOfQuad(
            double x1,
            double y1,
            double x2,
            double y2,
            double x3,
            double y3,
            double x4,
            double y4) {
        // Shoelace theorem
        return 0.5 * Math.abs(x1 * (y2 - y4) + x2 * (y3 - y1) + x3 * (y4 - y2) + x4 * (y1 - y3));
    }
}
