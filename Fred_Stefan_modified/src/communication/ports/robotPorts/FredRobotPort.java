package communication.ports.robotPorts;

import communication.ports.interfaces.ThreeWheelHolonomicRobotPort;
import communication.ports.interfaces.PropellerEquipedRobotPort;
import communication.ports.interfaces.RobotPort;

/**
 * Created by Simon Rovder
 */
public class FredRobotPort extends RobotPort implements PropellerEquipedRobotPort, ThreeWheelHolonomicRobotPort {

    public FredRobotPort(){
        super("pang");
    }

    @Override
    public void threeWheelHolonomicMotion(double back, double left, double right) {
        //this.sdpPort.commandSender("r", (int) front, (int) back, (int) left, (int) right);
        this.sdpPort.commandSender("r", (int) back, (int) left, (int) right);
    }

    @Override
    public void propeller(int spin) {
        if (spin>0) spin = 1;
        this.sdpPort.commandSender("kick", spin);
    }


}
