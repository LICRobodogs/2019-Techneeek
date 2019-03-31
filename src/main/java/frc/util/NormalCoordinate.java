package frc.util;
/**
 * normalized pixel coordinates, 0,0 is the center, positive right and up
 */
class NormalCoordinate extends Coordinate {
    public NormalCoordinate(double x, double y) {
        super(x, y);
    }

    public double getX_ViewPlaneCoord() {
        double x = getViewPlaneWidth() / 2 * getX();
        return x;
    }

    public double getY_ViewPlaneCoord() {
        double y = getViewPlaneHeight() / 2 * getY();
        return y;
    }

    public Coordinate getViewPlaneCoordinates() {
        return new Coordinate(getX_ViewPlaneCoord(), getY_ViewPlaneCoord());
    }

    private double getViewPlaneWidth() {
        double radianAngle = Math.toRadians(Constants.HORIZONTAL_FOV / 2);
        return 2.0 * Math.tan(radianAngle);
    }

    private double getViewPlaneHeight() {
        double radianAngle = Math.toRadians(Constants.VERTICAL_FOV / 2);
        return 2.0 * Math.tan(radianAngle);
    }
}