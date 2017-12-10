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
    static final double     FORWARD_SPEED = 0.2;
    static final double     BACKWARD_SPEED = 0.1;
    static final double     TURN_SPEED    = 0.1;

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
        String COLOR = colorSense();
        sleep(2000);
        handUp();

        if (COLOR == "RED")
        {
            goStraight(FORWARD_SPEED,.5);
            stopRobot();
        }
        else if (COLOR =="BLUE")
        {
            goBack(BACKWARD_SPEED,.2);
            stopRobot();
        }
        else
        {
            telemetry.addData("Nothing can be done",COLOR);
            telemetry.update();
        }

      //  telemetry.addData("Heading is",robot.gyroSensor.getHeading());
       // telemetry.update();
        sleep(5000);
        stopRobot();

/*
        goStraight(FORWARD_SPEED,.4);
        stopRobot();

         while (robot.gyroSensor.getHeading() < 358)
        {
            turnLeft(TURN_SPEED,1);
        }
        if (glyphPosition == ("LEFT"))
        {
            hLeft(INIT_FORWARD_SPEED,.2);
        }
        else if (glyphPosition.equals("RIGHT"))
        {
            hRight(INIT_FORWARD_SPEED,.2);
        }
        else if (glyphPosition.equals("CENTER"))
        {
            goBack(INIT_FORWARD_SPEED,.2);
        }
        else
        {
            stopRobot();
        }
        //goStraight(FORWARD_SPEED,1.0);
        //goBack(BACKWARD_SPEED,2.0);
       // turnRight(TURN_SPEED,0.15);
      //  goStraight(FORWARD_SPEED,.5);
        //stopRobot();
*/
    }

}
