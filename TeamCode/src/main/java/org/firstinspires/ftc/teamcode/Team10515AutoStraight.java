package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * This program will be used for autonomous Straight mode
 * It uses 2 color sensors.
 *      Rev Color Sensor is used to detect the platform color
 *      Color sensor is used to detect the jewel color
 * Based on the platform color the bot will move the appropriate jewel
 *      If RED  then  move  the blue jewel
 *      If BLUE then  move  the red  jewel
 *
 * Based on the vuforia reading the bot  will place glyph accordingly
 *
 */

@Autonomous(name="Auto: Straight", group="Team10515")
public class Team10515AutoStraight extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.6;
    static final double     BACKWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.6;
    static final double     HWHEEL_SPEED = 0.5;

    static final double ARM_UP_POWER = 0.5;
    static final double ARM_DOWN_POWER  = -0.5;

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

        String platformColor = colorSenseRev();
        sleep(100);
        String glyphPosition = vuforiaCapture();
        telemetry.addData("The position is" ,glyphPosition);
        telemetry.update();
        sleep(1000);
        liftUp(ARM_UP_POWER,2.0);
        stopRobot();
        handDown();
        sleep(100);
        String jewelColor = colorSense();
        sleep(500);

        if(platformColor.equals("RED"))
        {
            moveBlueJewel(jewelColor);
            sleep(1000);

            goBack(BACKWARD_SPEED,0.9);
            stopRobot();
            sleep(1000);

            //turnRight(TURN_SPEED,1.5);
            repositionBot(-180.0);
            stopRobot();
            sleep(1000);

         /*   goStraight(FORWARD_SPEED,.3);
            stopRobot();
            sleep(1000);*/
        }
        else
        {
            moveRedJewel(jewelColor);
            sleep(1000);

            goStraight(FORWARD_SPEED,1.0);
            stopRobot();
            sleep(1000);


            repositionBot(5.0);
            stopRobot();
            sleep(1000);

            /*goStraight(FORWARD_SPEED,.3);
            stopRobot();
            sleep(1000);*/
        }



       glyphPlacement(glyphPosition,platformColor);

    }


    private void glyphPlacement(String glyphPosition,String platformColor)
    {
        if (platformColor.equals("BLUE")) {
            if (glyphPosition.equals("LEFT") || glyphPosition.equals("UNKNOWN")) {
                hRight(HWHEEL_SPEED, .6);
            } else if (glyphPosition.equals("RIGHT")) {
                hRight(HWHEEL_SPEED, 1.5);
            } else if (glyphPosition.equals("CENTER")) {
                hRight(HWHEEL_SPEED, 1.0);
            }
        }else if (platformColor.equals("RED")){
            if (glyphPosition.equals("LEFT") || glyphPosition.equals("UNKNOWN")) {
                hLeft(HWHEEL_SPEED, .6);
            } else if (glyphPosition.equals("RIGHT")) {
                hLeft(HWHEEL_SPEED, 1.5);
            } else if (glyphPosition.equals("CENTER")) {
                hLeft(HWHEEL_SPEED, 1.0);
            }


        }
        goStraight(FORWARD_SPEED,.3 );
        stopRobot();
        robot.claw.setPosition(0);
    }

    private void moveBlueJewel(String jewelColor) {

        if (jewelColor.equals("RED")) {
          //  turnRight(TURN_SPEED, 0.2);
            goStraight(FORWARD_SPEED,0.4);
            stopRobot();
            handUp();
            goBack(BACKWARD_SPEED,0.5);
          //  turnLeft(TURN_SPEED, 0.2);
        } else if (jewelColor.equals("BLUE")) {
         //   turnLeft(TURN_SPEED, 0.2);
            goBack(BACKWARD_SPEED,0.4);
            stopRobot();
            handUp();
            goStraight(FORWARD_SPEED,0.5);
           // turnRight(TURN_SPEED, 0.2);

        } else {
            telemetry.addData("Nothing can be done", jewelColor);
            telemetry.update();
            handUp();
        }

        stopRobot();
    }

    private void moveRedJewel(String jewelColor) {

        if (jewelColor.equals("BLUE")) {
           // turnRight(FORWARD_SPEED, 0.2);
            goStraight(FORWARD_SPEED,0.4);
            stopRobot();
            handUp();
            goBack(BACKWARD_SPEED,0.4);
            //turnLeft(FORWARD_SPEED, 0.2);
        } else if (jewelColor.equals("RED")) {
           // turnLeft(BACKWARD_SPEED, 0.2);
            goBack(BACKWARD_SPEED,0.4);
            stopRobot();
            handUp();
            goStraight(FORWARD_SPEED,0.4);
            //turnRight(BACKWARD_SPEED, 0.2);

        } else {
            telemetry.addData("Nothing can be done", jewelColor);
            telemetry.update();
            handUp();
        }

        stopRobot();
    }
}
