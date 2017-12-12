package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * This program will go straight  hit the capball  and park the bot at the center vertex
 */

@Autonomous(name="Autonomous: Simplified Test", group="Pushbot")
public class SimplifiedAuto extends Team10515Base {

    static final double     INIT_FORWARD_SPEED = 0.1;
    static final double     FORWARD_SPEED = 0.4;
    static final double     BACKWARD_SPEED = 0.4;
    static final double     TURN_SPEED    = 0.1;

    static final double ARM_UP_POWER = -.03;
    public static final double ARM_DOWN_POWER  = 0.3;

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

        String glyphPosition = vuforiaCapture();
        telemetry.addData("The position is" ,glyphPosition);
        telemetry.update();
        sleep(2000);

        handDown();
        sleep(100);
        String color = colorSense();
        sleep(2000);

        if (color == "RED")
        {
            goStraight(FORWARD_SPEED,1.3);
            stopRobot();
            handUp();
        }
        else if (color =="BLUE")
        {
            goBack(BACKWARD_SPEED,.4);
            goStraight(BACKWARD_SPEED,1.7);
            stopRobot();
            handUp();
        }
        else
        {
            telemetry.addData("Nothing can be done",color);
            telemetry.update();
        }

        repositionBot();

        goStraight(FORWARD_SPEED,.3);
        stopRobot();



        //goStraight(FORWARD_SPEED,1.0);
        //goBack(BACKWARD_SPEED,2.0);
       // turnRight(TURN_SPEED,0.15);
      //  goStraight(FORWARD_SPEED,.5);
        //stopRobot();

    }
        private void glyphPlacement(String glyphPosition)
        {
            if (glyphPosition == ("LEFT"))
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
            goStraight(FORWARD_SPEED,.1);
            robot.claw.setPosition(0);
        }
}
