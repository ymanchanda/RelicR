package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This program will go straight  hit the capball  and park the bot at the center vertex
 */

@Autonomous(name="Auto: TURN Slide", group="Team10515")
public class Team10515AutoTurn_Slide extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARD_SPEED = 0.6;
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
        sleep(1000);
        liftUp(ARM_UP_POWER,.6);
        stopRobot();
        handDown();
        sleep(100);
        String jewelColor = colorSense();
        sleep(2000);

        if(platformColor.equals("RED")) {
            moveBlueJewel(jewelColor);
            sleep(1000);

            goBack(FORWARD_SPEED,1.3);
            stopRobot();
            sleep(1000);

            //turnLeft(TURN_SPEED,1.0);
            repositionBot(-270.0);
            stopRobot();
            sleep(1000);

            goStraight(FORWARD_SPEED,.3);
            stopRobot();
            sleep(1000);


        }else if(platformColor.equals("BLUE")) {
            moveRedJewel(jewelColor);
            sleep(1000);

            hRight(0.4,2.3);
            stopRobot();
            sleep(500);

            goStraight(FORWARD_SPEED,1.2);
            stopRobot();
            sleep(200);


                //turnLeft(TURN_SPEED,1.0);
            repositionBot(-270.0);
            stopRobot();
            sleep(1000);

//              goStraight(FORWARD_SPEED,.3);
//                stopRobot();
//                sleep(1000);
        }


        liftUp(ARM_UP_POWER,0.8);

        glyphPlacement(glyphPosition);


    }


    private void glyphPlacement(String glyphPosition)
    {
        if (glyphPosition.equals ("LEFT") || glyphPosition.equals("UNKNOWN"))
        {
            hRight(INIT_FORWARD_SPEED,.2);
        }
        else if (glyphPosition.equals("RIGHT"))
        {
            hRight(INIT_FORWARD_SPEED,.6);
        }
        else if (glyphPosition.equals("CENTER"))
        {
            hRight(INIT_FORWARD_SPEED,.4);

        }

        liftDown(ARM_DOWN_POWER,0.5);
        stopRobot();
        sleep(200);

        goStraight(FORWARD_SPEED,1.5);
        stopRobot();
        goBack(BACKWARD_SPEED,0.2);
        robot.claw.setPosition(0);
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
