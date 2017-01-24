package communication.ports.interfaces;

/**
 * Created by astabogdelyte on 24/01/2017.
 */
public interface ThreeWheelHolonomicRobotPort {
    void threeWheelHolonomicMotion(double back, double left, double right);
}
