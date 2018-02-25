package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This program will go straight  hit the capball  and park the bot at the center vertex
 */

@Autonomous(name="Auto: TURN Slide Sensor", group="Team10515")
public class Team10515AutoTurn_Slide_Sensor extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARD_SPEED = 0.6;
    static final double     HWHEEL_SPEED = 0.8;
    static final double     TURN_SPEED    = 0.1;

    static final double ARM_UP_POWER = 0.3;
    static final double ARM_DOWN_POWER  = -0.3;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.claw.setPosition(1);
        String platformColor = colorSenseRev();
        sleep(100);
        String glyphPosition = vuforiaCapture();
        telemetry.addData("The position is" ,glyphPosition);
        telemetry.update();
        sleep(100);
        liftUp(0.3,0.2);
        stopRobot();
        handDown();
        sleep(100);
        String jewelColor = colorSense();
        sleep(200);

        if(platformColor.equals("RED")) {
            moveBlueJewel(jewelColor);
            sleep(100);

            goBack(FORWARD_SPEED,1.3);
            stopRobot();
            sleep(100);

            repositionBot(-90.0 );
            stopRobot();
            sleep(100);

            goStraight(FORWARD_SPEED,.3);
            stopRobot();
            sleep(100);


        }else if(platformColor.equals("BLUE")) {
            moveRedJewel(jewelColor);
            sleep(100);

            hRight(0.4,2.0);
            stopRobot();
            sleep(500);

            goStraight(FORWARD_SPEED,1.2);
            stopRobot();
            sleep(200);

            repositionBotAntiClock(90.0);
            stopRobot();
            sleep(100);

        }

        glyphPlacement(glyphPosition,platformColor);

    }

    private void glyphPlacement(String glyphPosition,String platformColor)
    {
        if (platformColor.equals("BLUE")) {

            if (glyphPosition.equals("LEFT") || glyphPosition.equals("UNKNOWN")) {
                moveByRange(HWHEEL_SPEED,43.0);
            } else if (glyphPosition.equals("RIGHT")) {
                moveByRange(HWHEEL_SPEED,56.0);
            } else if (glyphPosition.equals("CENTER")){
                moveByRange(HWHEEL_SPEED,50.0);
            }
        }else if (platformColor.equals("RED")){
            if (glyphPosition.equals("RIGHT") || glyphPosition.equals("UNKNOWN")) {
                moveByRange(HWHEEL_SPEED,21.0);
            } else if (glyphPosition.equals("LEFT")) {
                moveByRange(HWHEEL_SPEED,34.0);
            } else if (glyphPosition.equals("CENTER")) {
                moveByRange(HWHEEL_SPEED,28.0);
            }


            repositionBot(-180.0);
            stopRobot();
            sleep(200);

            goBack(BACKWARD_SPEED,0.1);
            stopRobot();

        }


        stopRobot();
        sleep(200);
        goStraight(FORWARD_SPEED,1.5 );
        stopRobot();
        sleep(200);
        robot.claw.setPosition(0.5);
        stopRobot();
        goBack(BACKWARD_SPEED,0.1);
        stopRobot();

    }


    private void moveBlueJewel(String jewelColor) {

        if (jewelColor.equals("RED")) {
            turnRight(FORWARD_SPEED, 0.2);
            // goStraight(FORWARD_SPEED,0.5);
            stopRobot();
            handUp();
            turnLeft(FORWARD_SPEED, 0.2);
        } else if (jewelColor.equals("BLUE")) {
            turnLeft(BACKWARD_SPEED, 0.2);
            stopRobot();
            handUp();
            turnRight(BACKWARD_SPEED, 0.2);

        } else {
            telemetry.addData("Nothing can be done", jewelColor);
            telemetry.update();
            handUp();
        }

        stopRobot();
    }

    private void moveRedJewel(String jewelColor) {

        if (jewelColor.equals("BLUE")) {
            turnRight(FORWARD_SPEED, 0.2);
            // goStraight(FORWARD_SPEED,0.5);
            stopRobot();
            handUp();
            turnLeft(FORWARD_SPEED, 0.2);
        } else if (jewelColor.equals("RED")) {
            turnLeft(BACKWARD_SPEED, 0.2);
            stopRobot();
            handUp();
            turnRight(BACKWARD_SPEED, 0.2);

        } else {
            telemetry.addData("Nothing can be done", jewelColor);
            telemetry.update();
            handUp();
        }

        stopRobot();
    }
}
