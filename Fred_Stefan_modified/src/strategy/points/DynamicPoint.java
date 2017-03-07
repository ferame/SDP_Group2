package strategy.points;


import vision.tools.VectorGeometry;

/**
 * Created by Simon Rovder
 */
public interface DynamicPoint {
    void recalculate();
    int getX();
    int getY();

    boolean isBetweenPoints(VectorGeometry x, VectorGeometry y);
}
