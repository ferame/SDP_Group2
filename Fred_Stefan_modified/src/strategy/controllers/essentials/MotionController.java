package strategy.controllers.essentials;

import communication.ports.robotPorts.FredRobotPort;
import strategy.Strategy;
import strategy.actions.Behave;
import strategy.controllers.ControllerBase;
import strategy.navigation.NavigationInterface;
import strategy.navigation.Obstacle;
import strategy.navigation.potentialFieldNavigation.PotentialFieldNavigation;
import strategy.points.DynamicPoint;
import strategy.navigation.aStarNavigation.AStarNavigation;
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
    private boolean haveBall;

    private LinkedList<Obstacle> obstacles = new LinkedList<Obstacle>();

    public MotionController(RobotBase robot) {
        super(robot);
    }

    public enum MotionMode {
        ON, OFF
    }

    public void setMode(MotionMode mode) {
        this.mode = mode;
    }

    public void setTolerance(int tolerance) {
        this.tolerance = tolerance;
    }

    public int getTolerance() {
        return this.tolerance;
    }

    public void setDestination(DynamicPoint destination) {
        this.destination = destination;
    }

    public void setHeading(DynamicPoint dir) {
        this.heading = dir;
    }

    public void addObstacle(Obstacle obstacle) {
        this.obstacles.add(obstacle);
    }

    public void clearObstacles() {
        this.obstacles.clear();
    }

    public void perform() {

        if (this.mode == MotionMode.OFF) return;

        Robot us = Strategy.world.getRobot(RobotType.FRIEND_2);
        if (us == null) return;

        String distanceSensor = this.robot.port.getInput();
        if (distanceSensor.contains("0")) haveBall = true;
        else haveBall = false;
        //System.out.println("sensor: " + distanceSensor);

        // For now haveBall is always 0 and move function calls rotate (if ball is close to the robot), which then calls kick if we are facing the goal (this probably needs calibration)
//        if (haveBall == 0) {
//            move(us);
//        } else if (haveBall == 1) {
//            VectorGeometry destination = determineDestination(us, destination);;
//            rotate(us, destination, true);
        Boolean strategy = Behave.defend;
        if (strategy) defend(us);
        else attack(us);
        //attack(us);
        //defend(us);
    }

    private void attack(Robot us) {

        NavigationInterface navigation;

        VectorGeometry heading = null;
        VectorGeometry destination = null;

        /*double compassReading = 0.0;
        String input = this.robot.port.getInput();
        if (input.contains("Degrees")) {
            compassReading = Double.parseDouble(input.substring(0,6));
        }
        System.out.println(compassReading + "");*/

        if (this.destination != null) {

            this.destination.recalculate();

            destination = new VectorGeometry(this.destination.getX(), this.destination.getY());

            boolean intersects = false;


            for (Obstacle o : this.obstacles) {
                intersects = intersects || o.intersects(us.location, destination);
                //System.out.println("Check Obstacle");
            }

            for (Robot r : Strategy.world.getRobots()) {
                if (r != null && r.type != RobotType.FRIEND_2) {
                    intersects = intersects || VectorGeometry.vectorToClosestPointOnFiniteLine(us.location, destination, r.location).minus(r.location).length() < 30;
                    //System.out.println("Found potential obstacle");
                }
                //System.out.println("Check potential obstacle");

            }
//            System.out.println(us.location.distance(destination));
            // Robot is moving towards the ball. Why is "intersects" commented and what does it do ?
            if ( /*intersects ||  */ us.location.distance(destination) > 55 && !haveBall) {
                //StaticVariables.ballkicks = 0;
                navigation = new AStarNavigation();
//                navigation = new PotentialFieldNavigation();
                navigation.setHeading(destination);
                GUI.gui.searchType.setText("A*");
//                System.out.println("A* Prop down");
                for (int i = 0; i < 8; i++) {
                    ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(false);
                }

            } else if (us.location.distance(destination) > 21.5 && !haveBall /*- StaticVariables.ballkicks * 3*/ && us.location.distance(destination) < 55) {
                navigation = new AStarNavigation();
                //StaticVariables.haveBall = false;
                //navigation = new PotentialFieldNavigation();
                navigation.setHeading(destination);
                GUI.gui.searchType.setText("A*");
                System.out.println("Closer to ball: " + us.location.distance(destination));
                ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
                if (!haveBall) {
                    for (int i = 0; i < 8; i++) {
                        ((FredRobotPort) this.robot.port).propeller(100);
                    }
                }
                else {
                    for (int i = 0; i < 8; i++) {
                        ((FredRobotPort) this.robot.port).propeller(-100);
                    }
                }
            } else {
                ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
                destination = determineDestination(us, destination);
                System.out.println(haveBall);
                if (haveBall/* && StaticVariables.ballkicks == 0*/) {
                    for (int i = 0; i < 8; i++) {
                        ((FredRobotPort) this.robot.port).propeller(-100);
                    }
                    rotate(us, destination, true);
                    return;
                } else {
                    for (int i = 0; i < 8; i++) {
                        ((FredRobotPort) this.robot.port).propeller(100);
                    }
                    if (us.location.distance(destination) > 19.5){
                        navigation = new AStarNavigation();
                        //StaticVariables.haveBall = false;
                        //navigation = new PotentialFieldNavigation();
                        navigation.setHeading(destination);
                        GUI.gui.searchType.setText("A*");
                        System.out.println("Move before rotating " + us.location.distance(destination));

                    } else {
                        rotate(us, destination, false);
                        return;

                    }
                }
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


            navigation.setDestination(new VectorGeometry(destination.x, destination.y));


        } else {
//            System.out.println("Destination = null");
            return;
        }
        if (this.heading != null) {
            this.heading.recalculate();
            heading = new VectorGeometry(this.heading.getX(), this.heading.getY());
        } else heading = VectorGeometry.fromAngular(us.location.direction, 10, null);


        if (this.obstacles != null) {
            navigation.setObstacles(this.obstacles);
        }


        // Make sure that us not nul
        VectorGeometry force = navigation.getForce();

        if (force == null) {
            this.robot.port.stop();
            System.out.println("Force is null");
            return;
        }

        VectorGeometry robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
        VectorGeometry robotToPoint = VectorGeometry.fromTo(us.location, heading);
        double factor = 1;
        double rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);
        // Can throw null without check because null check takes SourceGroup into consideration.
        if (destination.distance(us.location) < 30) {
            factor = 0.7;
        }
        if (this.destination != null && us.location.distance(destination) < tolerance) {
            this.robot.port.stop();
            return;
        }
//        strategy.navigationInterface.draw();

        this.robot.drive.move(this.robot.port, us.location, force, rotation, factor);
    }

    // Rotates the robot towards the goal

    private void rotate(Robot us, VectorGeometry rotationDestination, Boolean kick) {
        NavigationInterface navigation;
        VectorGeometry destination = null;
        VectorGeometry heading = null;

        navigation = new PotentialFieldNavigation();
        navigation.setDestination(rotationDestination);

        navigation.setHeading(rotationDestination);

        /*if (!kick) {
            for (int i = 0; i < 3; i++) {
                ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
                ((FredRobotPort) this.robot.port).propeller(100);
            }
        } else {
            for (int i = 0; i < 3; i++) {
                ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
                ((FredRobotPort) this.robot.port).propeller(-100);
            }
        }*/

        /*if (this.heading != null) {
            this.heading.recalculate();
            heading = new VectorGeometry(this.heading.getX(), this.heading.getY());
        } else heading = VectorGeometry.fromAngular(us.location.direction, 10, null);*/

        VectorGeometry robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
        VectorGeometry robotToPoint = VectorGeometry.fromTo(us.location, rotationDestination);
        double factor = 1;
        double rotation = VectorGeometry.radToDeg(VectorGeometry.signedAngle(robotToPoint, robotHeading));
        if (rotation < 10 && rotation > -10 && kick) {
            double force = us.location.distance(rotationDestination);
            //System.out.println("kickforce" + force);
            kick(us, (int) Math.round(force));
        } else if (rotation < 10 && rotation > -10 && !kick) {
            /*try {
                //this.robot.drive.moveForward(this.robot.port);
                //Thread.sleep(200);
                catchBall(us);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }*/
            catchBall(us);
        } else {
            if (rotation < 0) this.robot.drive.rotate(this.robot.port, rotation);
            else this.robot.drive.rotate(this.robot.port, rotation);
        }


    }

    // Only the actual kicking happens here. it is called from rotate
    private void kick(Robot us, int power) {
//            ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
//            for (int i = 0; i < 5; i++) {
//                ((FredRobotPort) this.robot.port).propeller(-50);
//            }
        System.out.println("Kick");

        try {
            for(int i=0;i<8;i++) {
                this.robot.port.stop();
            }
            Thread.sleep(500);
            for (int i = 0; i < 8; i++) {
                ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
                ((FredRobotPort) this.robot.port).propeller(100);
            }
            Thread.sleep(500);

        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private void catchBall(Robot us) {
        System.out.println("Catch");
        for (int i = 0; i < 8; i++) {
            ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(true);
            ((FredRobotPort) this.robot.port).propeller(-100);
        }
        try {
            Thread.sleep(500);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private VectorGeometry determineDestination(Robot us, VectorGeometry oldDestination) {

        if (!haveBall) {
            System.out.println("Trying to find ball!");
            return oldDestination;
        }

        VectorGeometry enemyGoal = new VectorGeometry(Constants.PITCH_WIDTH / 2, 0);
        // This may break if we can't find the other friendly robot
        VectorGeometry alliedRobot = null;
        boolean canPassToGoal = true;
        boolean canPassToAlly = true;
        if (Strategy.world.getRobot(RobotType.FRIEND_1) != null){
            alliedRobot = Strategy.world.getRobot(RobotType.FRIEND_1).location;
            for (Obstacle o : this.obstacles) {
                canPassToAlly = canPassToAlly && !o.intersects(us.location, alliedRobot);
                //System.out.println("Check Obstacle");
            }
        }


        // Make Sure this does not cause infinite loop
        for (Obstacle o : this.obstacles) {
            canPassToGoal = canPassToGoal && !o.intersects(us.location, enemyGoal);
            //System.out.println("Check Obstacle");
        }


        VectorGeometry newDestination = null;
        if (us.location.distance(enemyGoal) < 120 && canPassToGoal) {
            newDestination = enemyGoal;
            System.out.println("Trying to score! normal");
        } else if (canPassToAlly && alliedRobot != null) {
            newDestination = alliedRobot;
            System.out.println("Trying to pass!");
        } else {
            newDestination = enemyGoal;
            System.out.println("Trying to score!");
        }
        return newDestination;

    }

    private void defend (Robot us) {

        if (this.destination == null) return;

        ((Fred) this.robot).PROPELLER_CONTROLLER.setActive(false);

        // go to closest defence point strategy
        /*VectorGeometry ourRobot = us.location;
        VectorGeometry ball = Strategy.world.getBall().location;
        if (ball == null) {
            Robot enemy = Strategy.world.getRobot(Strategy.world.getProbableBallHolder());
            ball = enemy.location;
            System.out.println("Ball is at enemy");
        }
        VectorGeometry ourGoal = new VectorGeometry(-Constants.PITCH_WIDTH / 2, 0);
        VectorGeometry robotToBall = ball.clone();
        robotToBall.minus(ourRobot);
        VectorGeometry ballToGoal = ourGoal.clone();
        ballToGoal.minus(ball);
        VectorGeometry negatedRobotToBall = robotToBall.clone();
        negatedRobotToBall.negate();
        double scalarProjection = VectorGeometry.dotProduct(negatedRobotToBall,ballToGoal)/ballToGoal.length();
        VectorGeometry projection = ballToGoal.multiply(scalarProjection/ballToGoal.length());
        VectorGeometry ourAim = robotToBall.plus(projection);
        NavigationInterface navigation = new AStarNavigation();
        navigation.setDestination(ourAim);
        navigation.setHeading(ball);*/

        // go to defence midpoint strategy
        VectorGeometry ourRobot = us.location;
        VectorGeometry ball = Strategy.world.getBall().location;
        if (ball == null) {
            Robot enemy = Strategy.world.getRobot(Strategy.world.getProbableBallHolder());
            ball = enemy.location;
            System.out.println("Ball is at enemy");
        }
        VectorGeometry ourGoal = new VectorGeometry(-Constants.PITCH_WIDTH / 2, 0);
        VectorGeometry ballToGoal = ourGoal.clone();
        ballToGoal.minus(ball).multiply(0.5);
        VectorGeometry ourAim = ball.clone().plus(ballToGoal);
        //VectorGeometry ourAim = temp.minus(ourRobot);
        NavigationInterface navigation = new AStarNavigation();
        navigation.setDestination(ourAim);
        navigation.setHeading(ball);

        VectorGeometry force = navigation.getForce();
        if (force == null) {
            this.robot.port.stop();
            System.out.println("Force is null");
            return;
        }

        VectorGeometry robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
        VectorGeometry robotToPoint = VectorGeometry.fromTo(us.location, ourAim);
        double rotation = VectorGeometry.signedAngle(robotToPoint, robotHeading);

        double factor = 1;

        if (us.location.distance(ourAim) < 20) factor = 0.5;

        if (us.location.distance(ourAim) > 10) {
            this.robot.drive.move(this.robot.port, us.location, force, rotation, factor);
        }
        else {this.robot.port.stop();}

    }

}
