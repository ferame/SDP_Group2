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
import vision.gui.MiscellaneousSettings;
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
    public static boolean ULTIMATE_DEFENCE = true;


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

        if (ULTIMATE_DEFENCE) {
            System.out.println("Hardcoded defence! Fuck those pancakes!");
            Robot us = Strategy.world.getRobot(this.robot.robotType);
            this.robot.drive.moveToCenter(this.robot.port, us.location.y < 0);
            if (Math.abs(us.location.y) < 15) {
                this.robot.port.stop();
                ULTIMATE_DEFENCE = false;
                MiscellaneousSettings.miscSettings.setUltimateDefenceToFalse();
            }
            return this.nextState;
        }

        String distanceSensor = this.robot.port.getInput();
        boolean res1 = distanceSensor.contains("0");
        boolean res2 = distanceSensor.contains("0");
        boolean res3 = distanceSensor.contains("0");
        if ((res1 && res2) || (res1 && res3) || (res2 && res3)) {defend = false; return this.nextState;}
        Ball ball = Strategy.world.getBall();
        if(ball == null){
            this.nextState = BehaviourEnum.DEFEND;
            defend = true;
        } else {
            VectorGeometry ourGoal = new VectorGeometry(-Constants.PITCH_WIDTH/2, 0);
            Robot us = Strategy.world.getRobot(this.robot.robotType);
            Robot friend = Strategy.world.getRobot(this.robot.robotType.FRIEND_1);
            Robot foe1 = Strategy.world.getRobot(this.robot.robotType.FOE_1);
            Robot foe2 = Strategy.world.getRobot(this.robot.robotType.FOE_2);

            if(us == null){
                System.out.println("Fuck this shit");
                defend = true;

            } else

                if(us.location.distance(ourGoal) > ball.location.distance(ourGoal)){
                System.out.println("GOAL IS OPEN !!! RETREATING !!");
                this.nextState = BehaviourEnum.SAFE;
                    defend = true;

            } else if(friend == null){
                System.out.println("No ally");
                this.nextState = BehaviourEnum.KICK;
                    if((foe1 != null && us.location.distance(ball.location) > foe1.location.distance(ball.location)) || (foe2 != null && us.location.distance(ball.location) > foe2.location.distance(ball.location))) {
                        System.out.println("An enemy is closer to the ball than me / defending");
                        this.nextState = BehaviourEnum.DEFEND;
                        defend = true;

                    }
                    else {
                        System.out.println("I'm closer to ball than enemies / kicking");
                        defend = false;
                    }

            } else if(us.location.distance(ball.location) < friend.location.distance(ball.location)){
                //if(WorldTools.isPointInEnemyDefenceArea2(ball.location)) {
                    if((foe1 != null && us.location.distance(ball.location) > foe1.location.distance(ball.location)) || (foe2 != null && us.location.distance(ball.location) > foe2.location.distance(ball.location))) {
                        System.out.println("An enemy is closer to the ball than me / defending");
                        this.nextState = BehaviourEnum.DEFEND;
                        defend = true;

                    }
                    else{
                        System.out.println("I'm closer to ball than enemies / kicking");
                        this.nextState = BehaviourEnum.KICK;
                        defend = false;
                    }
                /*} else{
                    System.out.println("kicking");
                    this.nextState = BehaviourEnum.KICK;
                    defend = false;
                }*/
            } else {
                    System.out.println("defending");
                this.nextState = BehaviourEnum.DEFEND;
                defend = true;
            }
        }
        return this.nextState;
    }

    /*protected BehaviourEnum getState2() {
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
    }*/
}

