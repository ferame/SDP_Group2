package strategy.controllers.essentials;

import communication.ports.robotPorts.FredRobotPort;
import strategy.Strategy;
import strategy.actions.other.Stop;
import strategy.controllers.ControllerBase;
import strategy.navigation.NavigationInterface;
import strategy.navigation.Obstacle;
import strategy.points.DynamicPoint;
import strategy.navigation.aStarNavigation.AStarNavigation;
import strategy.navigation.potentialFieldNavigation.PotentialFieldNavigation;
import strategy.points.DynamicPointBase;
import strategy.robots.Fred;
import strategy.robots.RobotBase;
import strategy.GUI;
import vision.Robot;
import vision.RobotType;
import vision.constants.Constants;
import vision.tools.VectorGeometry;

import java.util.LinkedList;

/**
 * Created by Simon Rovder
 */
public class MotionController extends ControllerBase {

    public MotionMode mode;
    private DynamicPoint heading = null;
    private DynamicPoint destination = null;

    private int tolerance;

    private LinkedList<Obstacle> obstacles = new LinkedList<Obstacle>();

    public MotionController(RobotBase robot) {
        super(robot);
    }

    public enum MotionMode{
        ON, OFF
    }

    public void setMode(MotionMode mode){
        this.mode = mode;
    }

    public void setTolerance(int tolerance){
        this.tolerance = tolerance;
    }

    public int getTolerance(){
        return this.tolerance;
    }

    public void setDestination(DynamicPoint destination){
        this.destination = destination;
    }

    public void setHeading(DynamicPoint dir){
        this.heading = dir;
    }

    public void addObstacle(Obstacle obstacle){
        this.obstacles.add(obstacle);
    }

    public void clearObstacles(){
        this.obstacles.clear();
    }

    public void perform(){
        if(this.mode == MotionMode.OFF) return;

        Robot us = Strategy.world.getRobot(RobotType.FRIEND_2);
        if(us == null) return;

        NavigationInterface navigation;

        VectorGeometry heading = null;
        VectorGeometry destination = null;



        if(this.destination != null){
            this.destination.recalculate();

            destination = new VectorGeometry(this.destination.getX(), this.destination.getY());

            boolean intersects = false;


            for(Obstacle o : this.obstacles){
                intersects = intersects || o.intersects(us.location, destination);
                //System.out.println("Check Obstacle");
            }

            for(Robot r : Strategy.world.getRobots()){
                if(r != null && r.type != RobotType.FRIEND_2){
                    intersects = intersects || VectorGeometry.vectorToClosestPointOnFiniteLine(us.location, destination, r.location).minus(r.location).length() < 30;
                    //System.out.println("Found potential obstacle");
                }
                //System.out.println("Check potential obstacle");

            }
            // Robot is moving towards the ball. Why is "intersects" commented and what does it do ?
            if( //intersects ||
                    ( us.location.distance(destination) > 40)){
                navigation = new AStarNavigation();
                GUI.gui.searchType.setText("A*");
                System.out.println("A* Prop down");

                ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(false);
                ((FredRobotPort) this.robot.port).propeller(0);
                ((FredRobotPort) this.robot.port).propeller(0);
                ((FredRobotPort) this.robot.port).propeller(0);

            } else
                // Robot is getting close to the ball
                {
                if( us.location.distance(destination) > 19 && us.location.distance(destination) < 40){
                    navigation = new AStarNavigation();
                    GUI.gui.searchType.setText("A*");
                    System.out.print("A* Prop up ");
                    System.out.println( us.location.distance(destination));
                    ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(true);
                    ((FredRobotPort) this.robot.port).propeller(-100);
                    ((FredRobotPort) this.robot.port).propeller(-100);
                    ((FredRobotPort) this.robot.port).propeller(-100);

                }
                // TODO if the ball is still near the robot after it has kicked move the robot and try to get the ball again.
                // Use some boolean getA
                else {
                    ((FredRobotPort) this.robot.port).propeller(50);
                    ((FredRobotPort) this.robot.port).propeller(50);
                    ((FredRobotPort) this.robot.port).propeller(50);
                    System.out.println("Yay");
                    //robot should face enemy goal
                    this.robot.MOTION_CONTROLLER.setHeading(DynamicPointBase.getEnemyGoalPoint());
                    // wait 0.5 seconds
                    try {
                        Thread.sleep(500);                 //1000 milliseconds is one second.
                    } catch(InterruptedException ex) {
                        Thread.currentThread().interrupt();
                    }
                    ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(true);
                    ((FredRobotPort) this.robot.port).propeller(-100);
                    ((FredRobotPort) this.robot.port).propeller(-100);
                    ((FredRobotPort) this.robot.port).propeller(-100);
                }
                // Potential field navigation is disabled for now
//                else {
//                    navigation = new PotentialFieldNavigation();
//                    ((FredRobotPort) this.robot.port).propeller(50);
//                    ((FredRobotPort) this.robot.port).propeller(50);
//                    ((FredRobotPort) this.robot.port).propeller(50);
//                    GUI.gui.searchType.setText("Potential Fields");
//                    System.out.println("Potential Field Navigation");
//                }

            }

            navigation.setDestination(new VectorGeometry(destination.x, destination.y));


        } else {
            return;
        }

        if(this.heading != null){
            this.heading.recalculate();
            heading = new VectorGeometry(this.heading.getX(), this.heading.getY());
        } else heading = VectorGeometry.fromAngular(us.location.direction, 10, null);



        if(this.obstacles != null){
            navigation.setObstacles(this.obstacles);
        }



        VectorGeometry force = navigation.getForce();
        if(force == null){
            this.robot.port.stop();
            return;
        }

        VectorGeometry robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
        VectorGeometry robotToPoint = VectorGeometry.fromTo(us.location, heading);
        double factor = 1;
        double rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);
        // Can throw null without check because null check takes SourceGroup into consideration.
        if(destination.distance(us.location) < 30){
            factor = 0.7;
        }
        if(this.destination != null && us.location.distance(destination) < tolerance){
            this.robot.port.stop();
            return;
        }


//        strategy.navigationInterface.draw();

        this.robot.drive.move(this.robot.port, us.location, force, rotation, factor);

    }
}
