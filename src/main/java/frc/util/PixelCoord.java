package frc.util;
/**
 * pixel coordinates, 0,0 is the upper-left, positive down and to the righ
 */
class PixelCoord extends Coordinate {
    public PixelCoord(double x, double y) {
        super(x, y);
    }

    /**
     * Normalize the X coordinate with respect to 320 horizontal fov
     */
    public double getX_normalized() {
        return (1 / 160) * (getX() - ((Constants.HORIZONTAL_FOV - 1) / 2));
    }

    /**
     * Normalize the Y coordinate with respect to 240 vertical fov
     */
    public double getY_normalized() {
        return (1 / 120) * (getY() - ((Constants.VERTICAL_FOV - 1) / 2));
    }

    public NormalCoordinate getNormalized() {
        return new NormalCoordinate(getX_normalized(), getY_normalized());
    }
    public double xAngle_toPixelLocation() {
        return angle_toPixelLocation(getNormalized().getViewPlaneCoordinates().getX());
    }

    public double yAngle_toPixelLocation() {
        return angle_toPixelLocation(getNormalized().getViewPlaneCoordinates().getY());
    }

    public double angle_toPixelLocation(double loc) {
        return Math.atan2(1, loc);
    }
}