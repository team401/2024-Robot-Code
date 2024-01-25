package frc.robot.utils;

public class FieldFinder {
    private class FieldLocations2024 {
        public static final double STAGE_RED_NEAR_SIDE_X = 0.0;
        public static final double STAGE_RED_NEAR_SIDE_Y = 0.0;
        public static final double STAGE_RED_LEFT_SIDE_X = 0.0;
        public static final double STAGE_RED_LEFT_SIDE_Y = 0.0;
        public static final double STAGE_RED_RIGHT_SIDE_X = 0.0;
        public static final double STAGE_RED_RIGHT_SIDE_Y = 0.0;

        public static final double STAGE_BLUE_NEAR_SIDE_X = 4.0;
        public static final double STAGE_BLUE_NEAR_SIDE_Y = 3.0;
        public static final double STAGE_BLUE_LEFT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_LEFT_SIDE_Y = 3.3;
        public static final double STAGE_BLUE_RIGHT_SIDE_X = 5.6;
        public static final double STAGE_BLUE_RIGHT_SIDE_Y = 2.7;

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

    public static FieldLocations whereAmI(double x, double y) {

        /*if (inTheTriangle(
                x,
                y,
                FieldLocations2024.STAGE_RED_NEAR_SIDE_X,
                FieldLocations2024.STAGE_RED_NEAR_SIDE_Y,
                FieldLocations2024.STAGE_RED_LEFT_SIDE_X,
                FieldLocations2024.STAGE_RED_LEFT_SIDE_Y,
                FieldLocations2024.STAGE_RED_RIGHT_SIDE_X,
                FieldLocations2024.STAGE_RED_RIGHT_SIDE_Y)) {
            return FieldLocations.RED_STAGE;
        } else*/ if (inTheTriangle(
                x,
                y,
                FieldLocations2024.STAGE_BLUE_NEAR_SIDE_X,
                FieldLocations2024.STAGE_BLUE_NEAR_SIDE_Y,
                FieldLocations2024.STAGE_BLUE_LEFT_SIDE_X,
                FieldLocations2024.STAGE_BLUE_LEFT_SIDE_Y,
                FieldLocations2024.STAGE_BLUE_RIGHT_SIDE_X,
                FieldLocations2024.STAGE_BLUE_RIGHT_SIDE_Y)) {
            return FieldLocations.BLUE_STAGE;
        } /* else if (x > FieldLocations2024.WING_RED_END_X) {
              return FieldLocations.RED_WING;
          } else if (x < FieldLocations2024.WING_BLUE_END_X) {
              return FieldLocations.BLUE_WING;
          }*/ else {
            return FieldLocations.MIDDLE;
        }
    }

    public static boolean willIHit(
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
        return false;
    }

    private static boolean inTheTriangle(
            double x, double y, double x1, double y1, double x2, double y2, double x3, double y3) {
        double area = areaOfTriangle(x1, y1, x2, y2, x3, y3);

        double a1 = areaOfTriangle(x, y, x2, y2, x3, y3);
        double a2 = areaOfTriangle(x1, y1, x, y, x3, y3);
        double a3 = areaOfTriangle(x1, y1, x2, y2, x, y);

        return area == a1 + a2 + a3;
    }

    private static double areaOfTriangle(
            double x1, double y1, double x2, double y2, double x3, double y3) {
        return 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    }
}
