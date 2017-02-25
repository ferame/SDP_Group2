package strategy.actions;

import strategy.GUI;
import strategy.Strategy;
import strategy.WorldTools;
import strategy.actions.other.DefendGoal;
import strategy.actions.other.GoToSafeLocation;
import strategy.actions.offense.OffensiveKick;
import strategy.robots.Fred;
import strategy.robots.RobotBase;
import vision.Ball;
import vision.Robot;
import vision.RobotType;
import vision.constants.Constants;
import vision.tools.VectorGeometry;

/**
 * Created by Simon Rovder
 */
enum BehaviourEnum{
    DEFEND, SHUNT, KICK, SAFE, EMPTY
}

/**
 * The main Action class. It basically plays the game.
 */
public class Behave extends StatefulActionBase<BehaviourEnum> {


    public static boolean RESET = true;
    public static boolean defend = true;


    public Behave(RobotBase robot){
        super(robot, null);
    }

    @Override
    public void enterState(int newState) {
        if(newState == 0){
            this.robot.setControllersActive(true);
        }
        this.state = newState;
    }


    @Override
    public void tok() throws ActionException {

        this.robot.MOTION_CONTROLLER.clearObstacles();
        if(this.robot instanceof Fred) ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(true);
        this.lastState = this.nextState;
        switch (this.nextState){
            case DEFEND:
                this.enterAction(new DefendGoal(this.robot), 0, 0);
                break;
            case KICK:
                this.enterAction(new OffensiveKick(this.robot), 0, 0);
                break;
            case SAFE:
                this.enterAction(new GoToSafeLocation(this.robot), 0, 0);
                break;

        }
    }

    @Override
    protected BehaviourEnum getState() {
        Ball ball = Strategy.world.getBall();
        if(ball == null){
            this.nextState = BehaviourEnum.DEFEND;
        } else {
            Robot us = Strategy.world.getRobot(this.robot.robotType);
            Robot friend = Strategy.world.getRobot(this.robot.robotType.FRIEND_1);
            System.out.println("In strategy chooser");

            if(us == null){
                System.out.println("Fuck this shit");

            } else if(friend == null){
                System.out.println("No ally");
                this.nextState = BehaviourEnum.KICK;
                defend = false;

            } else if(us.location.distance(ball.location) < friend.location.distance(ball.location)){
                System.out.println("kicking");
                this.nextState = BehaviourEnum.KICK;
                defend = false;
            } else {
                this.nextState = BehaviourEnum.DEFEND;
                defend = true;
            }
        }
        return this.nextState;
    }

    protected BehaviourEnum getState2() {
        Ball ball = Strategy.world.getBall();
        if(ball == null){
            this.nextState = BehaviourEnum.DEFEND;
        } else {
            Robot us = Strategy.world.getRobot(this.robot.robotType);
            if(us == null){
                System.out.println("I'm lost!!!");            } else {
                VectorGeometry ourGoal = new VectorGeometry(-Constants.PITCH_WIDTH/2, 0);
                if(us.location.distance(ourGoal) > ball.location.distance(ourGoal)){
                    this.nextState = BehaviourEnum.SAFE;
                } else {
                    if(Math.abs(ball.location.x) > Constants.PITCH_WIDTH/2 - 20 && Math.abs(ball.location.y) > Constants.PITCH_HEIGHT/2 - 20){
                        this.nextState = BehaviourEnum.SHUNT;
                    } else {
                        boolean canKick = true;
                        for(Robot r : Strategy.world.getRobots()){
                            if(r != null && r.type != RobotType.FRIEND_2 && r.velocity.length() < 1) canKick = canKick && r.location.distance(ball.location) > 50;
                        }
                        if(canKick && (this.lastState != BehaviourEnum.DEFEND || VectorGeometry.angle(ball.velocity, VectorGeometry.fromTo(ball.location, new VectorGeometry(-Constants.PITCH_WIDTH/2, 0))) > 2)){
                            this.nextState = BehaviourEnum.KICK;
                        } else {
                            this.nextState = BehaviourEnum.DEFEND;
                        }
                    }
                }
            }
        }
        return this.nextState;
    }
}

