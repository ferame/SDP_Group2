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

    private int haveBall = 1;

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
            if( /*intersects ||  us.location.distance(destination) > 50*/ false){
                haveBall = 0;
                navigation = new AStarNavigation();
                navigation.setHeading(destination);
                GUI.gui.searchType.setText("A*");
                //System.out.println("A* Prop down");

                ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(false);
                ((FredRobotPort) this.robot.port).propeller(0);
                ((FredRobotPort) this.robot.port).propeller(0);
                ((FredRobotPort) this.robot.port).propeller(0);

            } else
            // Robot is getting close to the ball
            {
//                if( us.location.distance(destination) > 22 && us.location.distance(destination) < 50){
                if (false) {
                    haveBall = 0;
                    navigation = new AStarNavigation();
                    navigation.setHeading(destination);
                    GUI.gui.searchType.setText("A*");
                    System.out.print("A* Prop up ");
                    System.out.println( us.location.distance(destination));
                    ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(true);
                    ((FredRobotPort) this.robot.port).propeller(100);
                    ((FredRobotPort) this.robot.port).propeller(100);
                    ((FredRobotPort) this.robot.port).propeller(100);

                }
                // TODO if the ball is still near the robot after it has kicked move the robot and try to get the ball again.
                // Use some boolean getA
                else {
                    haveBall = 1;
                    navigation = new AStarNavigation();
                    navigation.setHeading(destination);
                    navigation.setDestination(null);
                    ((FredRobotPort) this.robot.port).propeller(-50);
                    ((FredRobotPort) this.robot.port).propeller(-50);
                    ((FredRobotPort) this.robot.port).propeller(-50);
//                    System.out.println("Yay");
//                    this.robot.MOTION_CONTROLLER.setHeading(DynamicPointBase.getEnemyGoalPoint());
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
            if (haveBall == 1) {
                //navigation.setHeading(DynamicPointBase.getEnemyGoalPoint());
                //System.out.println("Facing enemy goal");
                navigation = new PotentialFieldNavigation();
                navigation.setDestination(new VectorGeometry(Constants.PITCH_WIDTH/2, 0));
                navigation.setHeading(new VectorGeometry(Constants.PITCH_WIDTH/2, 0));
            }
            else {
                System.out.println("I'm in else");
                navigation.setDestination(new VectorGeometry(destination.x, destination.y));
            }


        } else {
//            System.out.println("Destination = null");
            return;
        }
        //System.out.println("Heading: " + this.heading);
        if(this.heading != null){
            this.heading.recalculate();
            heading = new VectorGeometry(this.heading.getX(), this.heading.getY());
        } else heading = VectorGeometry.fromAngular(us.location.direction, 10, null);



        if(this.obstacles != null){
            navigation.setObstacles(this.obstacles);
        }


        // Make sure that us not nul
        VectorGeometry force = navigation.getForce();
        //System.out.println("Force: " + force);

        if(force == null){
            this.robot.port.stop();
            System.out.println("Force is null");
            return;
        }

        VectorGeometry robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
        VectorGeometry robotToPoint = VectorGeometry.fromTo(us.location, heading);
        double factor = 1;
        double rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);
        // Can throw null without check because null check takes SourceGroup into consideration.
        if(destination.distance(us.location) < 30){
            factor = 0.7;
        }else if (destination.distance(us.location) < 21){
            factor = 0.4;
        }
        if(this.destination != null && us.location.distance(destination) < tolerance){
            this.robot.port.stop();
            return;
        }
//        strategy.navigationInterface.draw();

        if (haveBall == 1){
            force =  new VectorGeometry(Constants.PITCH_WIDTH/2, 0);
            System.out.println("Begin");
            System.out.println(rotation);
            robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
             robotToPoint = VectorGeometry.fromTo(us.location, heading);
             rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);
            System.out.println(rotation);
            robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
            robotToPoint = VectorGeometry.fromTo(us.location, heading);
            rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);
            System.out.println(rotation);
            robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
            robotToPoint = VectorGeometry.fromTo(us.location, heading);
            rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);
            System.out.println(rotation);
            System.out.println("End");
//            System.out.println("robot heading " + robotHeading);
//            System.out.println("robot to point " + robotToPoint);

            if ( rotation < 0.1 && rotation > -0.05) {
                this.robot.port.stop();
                haveBall = 2;
            }
        }
        if (haveBall == 2){
//            force = new VectorGeometry(us.location.x, us.location.y);
            try {
                System.out.println("Kicking");
                ((Fred)this.robot).PROPELLER_CONTROLLER.setActive(true);
                ((FredRobotPort) this.robot.port).propeller(100);
                ((FredRobotPort) this.robot.port).propeller(100);
                ((FredRobotPort) this.robot.port).propeller(100);
                this.robot.port.stop();
                Thread.sleep(500);
                ((FredRobotPort) this.robot.port).propeller(0);
                ((FredRobotPort) this.robot.port).propeller(0);
                ((FredRobotPort) this.robot.port).propeller(0);
                haveBall = 0;
            } catch(InterruptedException ex) {
                System.out.print("ERROR");
                Thread.currentThread().interrupt();
            }
        }
        if (haveBall == 1){
            System.out.println("Rotating");
            this.robot.drive.rotate(this.robot.port, us.location, force, rotation, factor);
        }else {
            System.out.println("Moving");
            this.robot.drive.move(this.robot.port, us.location, force, rotation, factor);

        }
    }
}