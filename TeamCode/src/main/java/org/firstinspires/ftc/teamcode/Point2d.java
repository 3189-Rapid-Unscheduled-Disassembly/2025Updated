package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.List;

public class Point2d {
    public double x, y;

    public Point2d(double x, double y) {
        set(x, y);
    }

    public void set(double x, double y) {
        setX(x);
        setY(y);
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }


    @NonNull
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }

    public double distanceToOtherPoint(Point2d point2d) {
        return Math.hypot(point2d.x - this.x, point2d.y - this.y);
    }
    public double angleBetweenPoints(Point2d point2d) {
        double angle = Math.toDegrees(Math.atan2(point2d.y - this.y, point2d.x - this.x));
        if (angle < -90) {
            angle += 360;
        }
        return angle;
    }

    public double angleBetweenPoints0To180(Point2d point2d) {
        double angle = Math.toDegrees(Math.atan2(point2d.y - this.y, point2d.x - this.x));
        if (angle < 0) {
            angle += 360;
        }
        if (angle > 180) {
            angle -= 180;
        }
        return angle;
    }


    public Vector2d toVector2d() {
        return new Vector2d(x, y);
    }

    static public Point2d fromList(List<Double> list) {
        return new Point2d(list.get(0), list.get(1));
    }
}
