package communication.ports.interfaces;

/**
 * Created by Simon Rovder
 */
public interface ThreeWheelHolonomicRobotPort {
    void threeWheelHolonomicMotion(double back, double left, double right);
}
