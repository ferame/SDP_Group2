package strategy.actions.other;

import strategy.actions.ActionException;
import strategy.actions.ActionBase;
import strategy.robots.Fred;
import strategy.robots.RobotBase;

/**
 * Created by Simon Rovder
 */
public class Demo extends ActionBase {

    private int count = 0;

    public Demo(RobotBase robot) {
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
        //((ThreeWheelHolonomicRobotPort)this.robot.port).threeWheelHolonomicMotion(0,0,100,-100);
        this.state = newState;

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
