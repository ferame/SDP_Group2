package communication.ports.robotPorts;

import communication.ports.interfaces.KickerEquipedRobotPort;
import communication.ports.interfaces.RobotPort;
import communication.ports.interfaces.ThreeWheelHolonomicRobotPort;

/**
 * Created by astabogdelyte on 24/01/2017.
 */
public class AjjajourusRobotPort extends RobotPort implements KickerEquipedRobotPort, ThreeWheelHolonomicRobotPort {

    public AjjajourusRobotPort(){
        super("pang");
    }

    @Override
    public void threeWheelHolonomicMotion(double back, double left, double right) {
        this.sdpPort.commandSender("r", (int) back, (int) left, (int) right);
    }


    //TODO: what inputes does the kicker need to have?
    @Override
    public void kicker(int kick) {
        this.sdpPort.commandSender("kick", kick);
    }


}
