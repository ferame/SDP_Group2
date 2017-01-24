package strategy.drives;

import communication.ports.interfaces.ThreeWheelHolonomicRobotPort;
import communication.ports.interfaces.RobotPort;
import vision.tools.DirectedPoint;
import vision.tools.VectorGeometry;

/**
 * Created by astabogdelyte on 24/01/2017.
 */
public class ThreeWheelHolonomicDrive  implements DriveInterface{
    public int MAX_ROTATION = 30;
    public int MAX_MOTION = 200;

    public void move(RobotPort port, DirectedPoint location, VectorGeometry force, double rotation, double factor){
        assert(port instanceof ThreeWheelHolonomicRobotPort);

        VectorGeometry dir = new VectorGeometry();
        force.copyInto(dir).coordinateRotation(force.angle() - location.direction);
        factor = Math.min(1, factor);

        //TODO: how do those three wheels work
        double lim = this.MAX_MOTION - Math.abs(rotation* this.MAX_ROTATION *factor);

        double front = dir.y;
        double left = -dir.x;
        double back = -dir.y;
        double right = dir.x;
        double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), Math.max(Math.abs(front), Math.abs(back)));

        normalizer = lim/normalizer*factor;
        back  = back*normalizer + rotation * this.MAX_ROTATION;
        left  = left*normalizer + rotation * this.MAX_ROTATION;
        right = right*normalizer + rotation * this.MAX_ROTATION;

        ((ThreeWheelHolonomicRobotPort) port).threeWheelHolonomicMotion(back, left, right);

    }
}
