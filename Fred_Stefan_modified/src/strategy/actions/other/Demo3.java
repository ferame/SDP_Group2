package strategy.actions.other;

import communication.ports.interfaces.ThreeWheelHolonomicRobotPort;
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
            ((ThreeWheelHolonomicRobotPort)this.robot.port).threeWheelHolonomicMotion(255,255,255,255);
        } else {
            ((ThreeWheelHolonomicRobotPort)this.robot.port).threeWheelHolonomicMotion(-255,-255,-255,-255);
        }*/

        int MAX_ROTATION = 30;
        int MAX_MOTION = 200;
        int factor = 1;
        double x = 1, y = 0, w = 0;

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
        back = back/normalizer * 100;
        left = left/normalizer * 100;
        right = right/normalizer * 100;

        ((ThreeWheelHolonomicRobotPort)this.robot.port).threeWheelHolonomicMotion(back, left, -right);

        double compassReading = 0.0;

        String input = this.robot.port.getInput();
        if (input.contains("Degrees")) {
            compassReading = Double.parseDouble(input.substring(0,6));
        }
        System.out.println(compassReading + "");

        this.state = newState;
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
