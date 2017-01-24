package strategy.robots;

import communication.ports.robotPorts.FredRobotPort;
import strategy.controllers.ajjajourus.KickerController;
import strategy.drives.ThreeWheelHolonomicDrive;
import vision.RobotType;

/**
 * Created by astabogdelyte on 24/01/2017.
 */
public class Ajjajourus extends RobotBase{
    public final KickerController KICKER_CONTROLLER = new KickerController(this);

    public Ajjajourus(RobotType robotType){
        super(robotType, new FredRobotPort(), new ThreeWheelHolonomicDrive());
        this.controllers.add(this.KICKER_CONTROLLER);
    }


    @Override
    public void performManual() {

    }
}
