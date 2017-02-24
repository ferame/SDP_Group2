package vision.robotAnalysis.newRobotAnalysis;

import vision.Ball;
import vision.DynamicWorld;
import vision.Robot;
import vision.colorAnalysis.SDPColor;
import vision.robotAnalysis.RobotAnalysisBase;
import vision.RobotType;
import vision.robotAnalysis.RobotColorSettings;
import vision.spotAnalysis.approximatedSpotAnalysis.Spot;
import vision.tools.DirectedPoint;
import vision.tools.VectorGeometry;

import java.util.ArrayList;
import java.util.HashMap;
//import org.bytedeco.javacpp.presets.*;


/**
 * Created by Simon Rovder
 */
public class NewRobotAnalysis extends RobotAnalysisBase {


//    private SDPColor[] teamColor = {SDPColor.YELLOW, SDPColor.BLUE};
//    private SDPColor[] spotColor = {SDPColor.GREEN, SDPColor.PINK};


    public NewRobotAnalysis(){
        super();
    }


    @Override
    public void nextUndistortedSpots(HashMap<SDPColor, ArrayList<Spot>> spots, long time) {
        ArrayList<Spot> spotList;
        ArrayList<RobotPlate> plates = new ArrayList<RobotPlate>();

        /*
        ArrayList<Spot> greenSpots  = new ArrayList<Spot>();
        greenSpots.addAll(spots.get(SDPColor.GREEN));
        greenSpots.addAll(spots.get(SDPColor.GREEN_1));
        greenSpots.addAll(spots.get(SDPColor.GREEN_2));

        ArrayList<Spot> pinkSpots  = new ArrayList<Spot>();
        pinkSpots.addAll(spots.get(SDPColor.PINK));
        pinkSpots.addAll(spots.get(SDPColor.PINK_1));
        pinkSpots.addAll(spots.get(SDPColor.PINK_2));

        ArrayList<Spot> yellowSpots  = new ArrayList<Spot>();
        yellowSpots.addAll(spots.get(SDPColor.YELLOW));
        yellowSpots.addAll(spots.get(SDPColor.YELLOW_1));
        yellowSpots.addAll(spots.get(SDPColor.YELLOW_2));

        ArrayList<Spot> blueSpots  = new ArrayList<Spot>();
        blueSpots.addAll(spots.get(SDPColor.BLUE));
        blueSpots.addAll(spots.get(SDPColor.BLUE_1));
        blueSpots.addAll(spots.get(SDPColor.BLUE_2));
        */

        removeDuplicateColours(spots.get(SDPColor.GREEN));
        removeDuplicateColours(spots.get(SDPColor.PINK));

        PatternMatcher.patternMatch(spots.get(SDPColor.GREEN), plates);
        PatternMatcher.patternMatch(spots.get(SDPColor.PINK), plates);

        //PatternMatcher.patternMatch(greenSpots, plates);
        //PatternMatcher.patternMatch(pinkSpots, plates);

        PatternMatcher.singularValidate(spots.get(SDPColor.GREEN), plates);
        PatternMatcher.singularValidate(spots.get(SDPColor.PINK), plates);

        //PatternMatcher.singularValidate(greenSpots, plates);
        //PatternMatcher.singularValidate(pinkSpots, plates);

        PatternMatcher.removeInvalid(plates);

        PatternMatcher.teamAnalysis(plates, spots.get(SDPColor.YELLOW));
        PatternMatcher.teamAnalysis(plates, spots.get(SDPColor.BLUE));



        //PatternMatcher.teamAnalysis(plates, yellowSpots);
        //PatternMatcher.teamAnalysis(plates, blueSpots);

        DynamicWorld world = new DynamicWorld(time);
        Robot r;

        for(RobotPlate plate : plates){
            if(!plate.hasTeam()){
                plate.setTeam(RobotColorSettings.ASSUME_YELLOW ? SDPColor.YELLOW : SDPColor.BLUE);
            }
            r = plate.toRobot();
            world.setRobot(r);
        }

        spotList = spots.get(SDPColor._BALL);

        for(int i = 0; i < spotList.size(); i++){
            Spot s = spotList.get(i);
            if(PatternMatcher.isBotPart(plates, s)){
                spotList.remove(i);
                i--;
            }
        }


        world.robotCount = world.getRobots().size();


        Ball ball;
        Ball oldBall = null;
        long timeDelta = 0;
        if(lastKnownWorld != null){
            oldBall = lastKnownWorld.getBall();
            timeDelta = (time - lastKnownWorld.getTime())/1000000;
        }

        if(timeDelta == 0) timeDelta = 1;

        if(spotList.size() > 0){
            ball = new Ball();
            ball.location = spotList.get(0);
            world.setBall(ball);
        }
        ball = world.getBall();

        if(lastKnownWorld != null && world.robotChangeDelay != 0 && lastKnownWorld.getProbableBallHolder() != null){

            world.setBall(null);
            world.setProbableBallHolder(lastKnownWorld.getProbableBallHolder());
            world.setLastKnownBall(lastKnownWorld.getLastKnownBall());
            if(world.robotCount == lastKnownWorld.robotCount){
                world.robotChangeDelay = lastKnownWorld.robotChangeDelay - 1;
            } else {
                world.robotChangeDelay = 20;
            }
        } else {
            if(this.lastKnownWorld != null){
                Ball lastKnownBall = this.lastKnownWorld.getBall();
                if(lastKnownBall != null){
                    if(ball == null){
                        Robot closest = null;
                        for(Robot robot : world.getRobots()){
                            if(closest == null) closest = robot;
                            else {
                                if(lastKnownBall.location.distance(closest.location) > lastKnownBall.location.distance(robot.location)){
                                    closest = robot;
                                }
                            }
                        }

                        if(closest != null && closest.location.distance(lastKnownBall.location) < 30){
                            Ball newBall = new Ball();
                            newBall.location = closest.location.clone();
                            world.setBall(newBall);
                            world.setProbableBallHolder(closest.type);
                        } else {
                            world.setBall(lastKnownBall.clone());
                        }
                    }
                }
            }
        }

        if(lastKnownWorld != null && world.getBall() != null && lastKnownWorld.getBall() != null){
            VectorGeometry velocity = VectorGeometry.fromTo(lastKnownWorld.getBall().location, world.getBall().location);
            velocity.setLength(velocity.length()/timeDelta);
            world.getBall().velocity = velocity;
        } else {
            if(world.getBall() != null) world.getBall().velocity = new VectorGeometry(0,0);
        }

        if(lastKnownWorld != null){
            for(RobotType rt : RobotType.values()){
                Robot old = lastKnownWorld.getRobot(rt);
                Robot newR = world.getRobot(rt);
                if(newR != null){
                    if(old != null){
                        newR.velocity = newR.location.clone();
                        newR.velocity.minus(old.location);
                        newR.velocity.direction = (newR.location.direction - old.location.direction)/timeDelta;
                        newR.velocity.setLength(newR.velocity.length() / timeDelta);
                    } else {
                        newR.velocity = new DirectedPoint(0,0,0);
                    }
//                    System.out.println(newR.velocity);
                }
            }
        }


        for(Robot toUndistort : world.getRobots()){
            if(toUndistort != null){
                // This part moves the strategy.robots closer to the center of the pitch. It compensates for the height
                // of the robot (15 cm)
                toUndistort.location.setLength(toUndistort.location.length() * (1 - 20.0/250));
            }
        }
        this.informListeners(world);
    }

    private void removeDuplicateColours(ArrayList<Spot> spots) {
        for (int i = 0; i < spots.size(); i++) {
            for (int j = i + 1; j < spots.size(); j++) {
                if (VectorGeometry.distance(spots.get(i), spots.get(j)) <= 4) {
                    if (Math.abs(spots.get(i).magnitude - 80) > Math.abs(spots.get(j).magnitude - 80)){
                        Spot buff = spots.get(j);
                        spots.set(j, spots.get(i));
                        spots.set(i, buff);
                    }
                    spots.remove(j);
                }
            }

        }

    }
}
