package strategy.actions.other;

import strategy.Strategy;
import strategy.actions.ActionBase;
import strategy.actions.ActionException;
import strategy.robots.Fred;
import strategy.robots.RobotBase;
import vision.Robot;
import vision.RobotType;
import vision.constants.Constants;
import vision.tools.VectorGeometry;

/**
 * Created by Simon Rovder
 */
public class Demo2 extends ActionBase {

    private int count = 0;

    public Demo2(RobotBase robot) {
        super(robot);
        assert(robot instanceof Fred);
        this.rawDescription = " Demo Action";
    }

    @Override
    public void enterState(int newState) {
        if (count == 3) return;
        this.robot.MOTION_CONTROLLER.setActive(false);
        /*if(newState == 0){
            ((ThreeWheelHolonomicRobotPort)this.robot.port).threeWheelHolonomicMotion(255,255,255,255);
        } else {
            ((ThreeWheelHolonomicRobotPort)this.robot.port).threeWheelHolonomicMotion(-255,-255,-255,-255);
        }*/
        Robot us = Strategy.world.getRobot(RobotType.FRIEND_2);
        VectorGeometry robotHeading = VectorGeometry.fromAngular(us.location.direction, 10, null);
        VectorGeometry enemyGoal = new VectorGeometry(Constants.PITCH_WIDTH / 2, 0);
        VectorGeometry robotToGoal = enemyGoal.clone().minus(us.location);
        double angleToRotate = VectorGeometry.radToDeg(VectorGeometry.angle(robotHeading, robotToGoal));

        double compassReading = 0.0;
        String input = this.robot.port.getInput();
        System.out.println(input);
        compassReading = Double.parseDouble(input.substring(16, input.length()));

        double rotationGoal = (compassReading + angleToRotate)%360;
        System.out.println("goal: " + rotationGoal);

        while (true) {

            if (Math.abs(rotationGoal - compassReading) > 5) {
                double direction1 = compassReading - rotationGoal;
                if (direction1 < 0) direction1 += 360;
                double direction2 = rotationGoal - compassReading;
                if (direction2 < 0) direction1 += 360;
                if (direction1<direction2) this.robot.drive.rotate(this.robot.port, -100);
                else this.robot.drive.rotate(this.robot.port, 100);
            }
            else break;

            input = this.robot.port.getInput();
            System.out.println(input);
            compassReading = Double.parseDouble(input.substring(16, input.length()));

            System.out.println(compassReading + "");
        }

        count = 3;


        /*VectorGeometry robotToPoint = VectorGeometry.fromTo(us.location, dest);
        double factor = 1;
        double rotation = VectorGeometry.radToDeg(VectorGeometry.signedAngle(robotToPoint, robotHeading));*/

        /*StaticVariables.recentRotations[0] = StaticVariables.recentRotations[1];
        StaticVariables.recentRotations[1] = StaticVariables.recentRotations[2];
        StaticVariables.recentRotations[2] = StaticVariables.recentRotations[3];
        StaticVariables.recentRotations[3] = StaticVariables.recentRotations[4];
        StaticVariables.recentRotations[4] = rotation;

        double[] temp = new double[5];
        for (int i=0;i<5;i++)
            temp[i] = StaticVariables.recentRotations[i];
        Arrays.sort(temp);
        if (temp[0] != 0.0 && temp[1] != 0.0) rotation = temp[2];

        System.out.println(StaticVariables.recentRotations[0] + " " + StaticVariables.recentRotations[1] + " " + StaticVariables.recentRotations[2] + " " + StaticVariables.recentRotations[3] + " " + StaticVariables.recentRotations[4]);*/

        /*System.out.println("Rotation " + rotation + " ");

        if (rotation < 10 && rotation > -10) {
            this.robot.port.stop();
            this.robot.port.stop();
            this.robot.port.stop();
            this.robot.port.stop();
            this.robot.port.stop();
            this.robot.port.stop();
            count = 3;
        } else {
            if (rotation<0) this.robot.drive.rotate(this.robot.port, -factor);
            else this.robot.drive.rotate(this.robot.port, factor);
        }*/

    }

    @Override
    public void tok() throws ActionException {
        //if(count > 3) throw new ActionException(true, false);
        //count++;
        if (count==3) return;
        if(this.state == 0) this.enterState(1);
        else this.enterState(0);
    }
}
