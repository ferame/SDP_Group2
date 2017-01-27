package strategy.drives;

import communication.ports.interfaces.FourWheelHolonomicRobotPort;
import communication.ports.interfaces.RobotPort;
import vision.tools.DirectedPoint;
import vision.tools.VectorGeometry;

/**
 * Created by Simon Rovder
 */

public class FourWheelHolonomicDrive implements DriveInterface{

    public int MAX_ROTATION = 30;
    public int MAX_MOTION = 100;

    public void move(RobotPort port, DirectedPoint location, VectorGeometry force, double rotation, double factor){
        assert(port instanceof FourWheelHolonomicRobotPort);

        VectorGeometry dir = new VectorGeometry();
        force.copyInto(dir).coordinateRotation(force.angle() - location.direction);

        double[][] a = new double[3][3];
        a[0][0] = 0.58; a[0][1] = -0.33; a[0][2] = 0.33;
        a[1][0] = -0.58; a[1][1] = -0.33; a[1][2] = 0.33;
        a[2][0] = 0; a[2][1] = 0.67; a[2][2] = 0.33;
        
        double front = 0;
        double back = a[2][0] * dir.x + a[2][1] * dir.y + a[2][2] * rotation;
        double left = a[1][0] * dir.x + a[1][1] * dir.y + a[1][2] * rotation;
        double right = a[0][0] * dir.x + a[0][1] * dir.y + a[0][2] * rotation;

        double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), Math.max(Math.abs(front), Math.abs(back)));
        front = front / normalizer * this.MAX_MOTION;
        back  = back / normalizer * this.MAX_MOTION;
        left  = left / normalizer * this.MAX_MOTION;
        right = right / normalizer * this.MAX_MOTION;

        /*
        factor = Math.min(1, factor);
        double front = dir.y;
        double left = -dir.x;
        double back = -dir.y;
        double right = dir.x;
        double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), Math.max(Math.abs(front), Math.abs(back)));
        double lim = this.MAX_MOTION - Math.abs(rotation* this.MAX_ROTATION *factor);
        normalizer = lim/normalizer*factor;
        front = front*normalizer + rotation * this.MAX_ROTATION;
        back  = back*normalizer + rotation * this.MAX_ROTATION;
        left  = left*normalizer + rotation * this.MAX_ROTATION;
        right = right*normalizer + rotation * this.MAX_ROTATION;*/


        ((FourWheelHolonomicRobotPort) port).fourWheelHolonomicMotion(front, back, left, right);

    }
}
