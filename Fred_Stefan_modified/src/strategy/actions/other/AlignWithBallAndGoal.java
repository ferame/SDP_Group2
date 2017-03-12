package strategy.actions.other;

import strategy.Strategy;
import strategy.actions.ActionBase;
import strategy.actions.ActionException;
import strategy.points.DynamicPoint;
import strategy.robots.RobotBase;
import vision.Ball;
import vision.Robot;
import vision.RobotType;
import vision.constants.Constants;
import vision.tools.VectorGeometry;


public class AlignWithBallAndGoal extends ActionBase {
    public AlignWithBallAndGoal (RobotBase robot){
        super(robot);
    }
    public AlignWithBallAndGoal (RobotBase robot, DynamicPoint point) {
        super(robot,point);
        this.rawDescription = " Align ourself with goal and ball";
    }
    @Override
    public void enterState(int newState) {
        if (aligned()==false){

            this.robot.MOTION_CONTROLLER.setDestination(this.point);
            this.robot.MOTION_CONTROLLER.setHeading(this.point);
            System.out.println("Setting the destination");
        }
        this.state = newState;
    }

    @Override
    public void tok() throws ActionException {
        Robot me = Strategy.world.getRobot(RobotType.FRIEND_2);
        Ball ball = Strategy.world.getBall();
        VectorGeometry ourGoal = new VectorGeometry(-Constants.PITCH_WIDTH/2, 0);

        if(me == null){
            this.enterState(0);
            return;
        }

        if((VectorGeometry.distance(this.point.getX(), this.point.getY(), me.location.x, me.location.y) < 10))
            if (this.point.isBetweenPoints(ourGoal,ball.location))
                this.enterState(2);
        else {
            if(this.state == 0){
                System.out.println("here");
                this.enterState(1);
            }
        }
    }

    public static boolean aligned(){

        Robot me  = Strategy.world.getRobot(RobotType.FRIEND_2);
        Ball ball = Strategy.world.getBall();
        VectorGeometry ourGoal = new VectorGeometry(-Constants.PITCH_WIDTH/2, 0);
        VectorGeometry v1 = new VectorGeometry(ball.location , me.location);
        VectorGeometry v2 = new VectorGeometry(ourGoal , me.location);
        if(me == null || ball == null || ourGoal == null) return false;
        return v1.isCollinear(v2);
    }
}
