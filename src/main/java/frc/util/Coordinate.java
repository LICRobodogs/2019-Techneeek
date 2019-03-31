package frc.util;

/**
 * Helper class in case we need to start translating coordinates or mapping
 * coordinates onto virtual plane --Purpose of 3 classes is remapping raw pixels
 * into coordinate locations then determining their location on a virtual plan
 * and uses the two computed coordinate locations to determine angles
 */
public class Coordinate {
    private double x, y;

    public Coordinate() {
        this(0.0, 0.0);
    }

    public Coordinate(double x, double y) {
        set(x, y);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        set(x, this.y);
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        set(this.x, y);
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }
}