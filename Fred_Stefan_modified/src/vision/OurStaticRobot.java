package vision;

import vision.tools.DirectedPoint;

/**
 * Created by Simon Rovder
 */
public class OurStaticRobot {
    public DirectedPoint location;
    public DirectedPoint velocity;
    public RobotType type;
    public RobotAlias alias;
    public double angle;
    public int hasBall;
    public int consecutiveKicks;


//    public Robot(){
//
//    }

    public void updateRobot(Robot robot) {
        this.location = robot.location;
        this.velocity = robot.velocity;
        this.type = robot.type;
        this.alias = robot.alias;
    }
}
