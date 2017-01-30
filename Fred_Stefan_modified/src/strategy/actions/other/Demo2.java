package strategy.actions.other;

import communication.ports.interfaces.FourWheelHolonomicRobotPort;
import communication.ports.robotPorts.FredRobotPort;
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
        ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(true);
        ((FredRobotPort) this.robot.port).propeller(50);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        ((FredRobotPort) this.robot.port).propeller(0);
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
