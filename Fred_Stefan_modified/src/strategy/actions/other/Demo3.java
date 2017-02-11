package strategy.actions.other;

import communication.ports.interfaces.FourWheelHolonomicRobotPort;
import strategy.actions.ActionBase;
import strategy.actions.ActionException;
import strategy.robots.Fred;
import strategy.robots.RobotBase;

/**
 * Created by Simon Rovder
 */
public class Demo3 extends ActionBase {

    private int count = 0;

    public Demo3(RobotBase robot) {
        super(robot);
        assert(robot instanceof Fred);
        this.rawDescription = " Demo Action";
    }

    @Override
    public void enterState(int newState) {
        this.robot.MOTION_CONTROLLER.setActive(false);
        /*if(newState == 0){
            ((FourWheelHolonomicRobotPort)this.robot.port).fourWheelHolonomicMotion(255,255,255,255);
        } else {
            ((FourWheelHolonomicRobotPort)this.robot.port).fourWheelHolonomicMotion(-255,-255,-255,-255);
        }*/

        int MAX_ROTATION = 30;
        int MAX_MOTION = 200;
        int factor = 1;
        double x = 0, y = 1, w = 0;

        double[][] a = new double[3][3];
        a[0][0] = 0.58; a[0][1] = -0.33; a[0][2] = 0.33;
        a[1][0] = -0.58; a[1][1] = -0.33; a[1][2] = 0.33;
        a[2][0] = 0; a[2][1] = 0.67; a[2][2] = 0.33;

        /*double[][] a = new double[3][3];
        a[0][0] = -0.5; a[0][1] = 0.8660254; a[0][2] = 1.0;
        a[1][0] = -0.5; a[1][1] = -0.8660254; a[1][2] = 1.0;
        a[2][0] = 1.0; a[2][1] = 0.0; a[2][2] = 1.0;*/

        double front = 0;
        double back = a[2][0] * x + a[2][1] * -y + a[2][2] * w;
        double left = a[1][0] * x + a[1][1] * -y + a[1][2] * w;
        double right = a[0][0] * x + a[0][1] * -y + a[0][2] * w;

        /*double lim = MAX_MOTION - Math.abs(MAX_ROTATION * factor);
        double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), Math.max(Math.abs(front), Math.abs(back)));
        normalizer = lim/normalizer*factor;
        front = front*normalizer + w * MAX_ROTATION;
        back  = back*normalizer + w * MAX_ROTATION;
        left  = left*normalizer + w * MAX_ROTATION;
        right = right*normalizer + w * MAX_ROTATION;*/

        double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), Math.max(Math.abs(front), Math.abs(back)));
//        back = back/normalizer * 100;
//        back = adjustingMotors(1, back / normalizer * 100);
        back = 0;
        left = adjustingMotors(2, left/normalizer * -100);
        right = adjustingMotors(3, right/normalizer * 100);

        ((FourWheelHolonomicRobotPort)this.robot.port).fourWheelHolonomicMotion(front, back, left, -right);

        this.state = newState;
    }

    //motors: front - 0, back - 1, left - 2, right - 3
    public double adjustingMotors(int motor, double power){
        System.out.println("adjusting motor " + Integer.toString(motor));
        System.out.println("power is " + power);
        int[] powers = new int[4];
//        powers[0] = 0;
        powers[1] = 0;
        powers[2] = 0;
        powers[3] = 0;
        boolean weakest = false;
        if (motor == 0) {
            weakest = true;
        }
        if(power > 93 && !weakest){
            System.out.println("power > 93 && !weakest");
            System.out.println(power - powers[motor]);
            return power - powers[motor];
        } else if (power > 93 && weakest){
            System.out.println("power > 93 && weakest");
            System.out.println(power);
            return power;
        } else if (!weakest){
            System.out.println("!weakest");
            System.out.println(power);
            return power;
        } else {
            System.out.println("weakest");
            System.out.println(power + powers[motor]);
            return power + powers[motor];
        }
    }

    @Override
    public void tok() throws ActionException {
        if(count > 3) throw new ActionException(true, false);
        count++;
        if(this.state == 0) this.enterState(1);
        else this.enterState(0);
        this.delay(2000);
    }
}
