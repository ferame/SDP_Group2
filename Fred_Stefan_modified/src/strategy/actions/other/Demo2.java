package strategy.actions.other;

import communication.ports.interfaces.FourWheelHolonomicRobotPort;
import strategy.actions.ActionBase;
import strategy.actions.ActionException;
import strategy.robots.Fred;
import strategy.robots.RobotBase;

/**
 * Created by Simon Rovder
 */
public class Demo2 extends ActionBase {

    private int count = 0;

    public Demo2(RobotBase robot) {
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
        ((FourWheelHolonomicRobotPort)this.robot.port).fourWheelHolonomicMotion(0,100,100,-100);
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
