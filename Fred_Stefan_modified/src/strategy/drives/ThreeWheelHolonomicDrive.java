package strategy.drives;

import communication.ports.interfaces.ThreeWheelHolonomicRobotPort;
import communication.ports.interfaces.RobotPort;
import vision.tools.DirectedPoint;
import vision.tools.VectorGeometry;

/**
 * Created by Simon Rovder
 */

public class ThreeWheelHolonomicDrive implements DriveInterface{

    public int MAX_ROTATION = 90;
    public int MAX_MOTION = 100;

    public void move(RobotPort port, DirectedPoint location, VectorGeometry force, double rotation, double factor){
        assert(port instanceof ThreeWheelHolonomicRobotPort);

        VectorGeometry dir = new VectorGeometry();
        force.copyInto(dir).coordinateRotation(force.angle() - location.direction);

        double[][] a = new double[3][3];
        a[0][0] = 0.58; a[0][1] = -0.33; a[0][2] = 0.33;
        a[1][0] = -0.58; a[1][1] = -0.33; a[1][2] = 0.33;
        a[2][0] = 0; a[2][1] = 0.67; a[2][2] = 0.33;
        double back = a[2][0] * dir.x + a[2][1] * -dir.y + a[2][2] * rotation;
        double left = a[1][0] * dir.x + a[1][1] * -dir.y + a[1][2] * rotation;
        double right = a[0][0] * dir.x + a[0][1] * -dir.y + a[0][2] * rotation;

        double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), Math.max(0, Math.abs(back)));
        back  = adjustingMotors(1, back / normalizer * this.MAX_MOTION);
        left  = adjustingMotors(2,left / normalizer * this.MAX_MOTION);
        right = adjustingMotors(3,right / normalizer * this.MAX_MOTION);

        ((ThreeWheelHolonomicRobotPort) port).threeWheelHolonomicMotion(back, left, -right);

    }
    //motors: front - 0, back - 1, left - 2, right - 3
    public double adjustingMotors(int motor, double power){
        int[] powers = new int[4];
        powers[0] = 0;
        powers[1] = 0;
        powers[2] = 0;
        powers[3] = 0;
        boolean weakest = false;
        if (motor == 2) {
            weakest = true;
        }
        if(power > 93 && !weakest){
            return power - powers[motor];
        } else if (power > 93 && weakest){
            return power;
        } else if (!weakest){
            return power;
        } else {
            return power + powers[motor];
        }
    }

    //TODO rotate slower when closer to the goal
    public void rotate(RobotPort port, double force){
        assert(port instanceof ThreeWheelHolonomicRobotPort);

        double back = 40;
        double left = 40;
        double right = 40;

        if (force<0) {
            back *= -1;
            right *= -1;
            left *= -1;
        }

        ((ThreeWheelHolonomicRobotPort) port).threeWheelHolonomicMotion(back, left, -right);

    }

    public void moveForward(RobotPort port){
        assert(port instanceof ThreeWheelHolonomicRobotPort);

        double back = 0;
        double left = 50;
        double right = 50;

        System.out.println("Moving forward!");

        ((ThreeWheelHolonomicRobotPort) port).threeWheelHolonomicMotion(back, left, -right);

    }

}
