package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.robotcore.util.Range;


public abstract class RR10515Base extends LinearOpMode {

    /* Declare OpMode members. */
    RR10515HardwareMap robot = new RR10515HardwareMap();   // Use our Team 10515 hardware
    ElapsedTime runtime = new ElapsedTime();

    public static final String TAG = "Vuforia VuMark Sample";
    //public static final double ARM_DOWN_POWER  = 0.3;
    //public static final double ARM_UP_POWER  = -0.3;


    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    public void goStraight(double speed, double period) {

        robot.BLeftMotor.setPower(speed);
        robot.BRightMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //  telemetry.update();
        }
    }
   /* public void driveStraightDistance(int tenthsOfIn, int masterPower)
    {
        int tickGoal = (42 * tenthsOfIn) / 10;

        //This will count up the total encoder ticks despite the fact that the encoders are constantly reset.
        int totalTicks = 0;

        //Initialise slavePower as masterPower - 5 so we don't get huge error for the first few iterations. The
        //-5 value is based off a rough guess of how much the motors are different, which prevents the robot from
        //veering off course at the start of the function.
        int slavePower = masterPower - 5;

        int error = 0;

        int kp = 5;

        SensorValue[leftEncoder] = 0;
        SensorValue[rightEncoder] = 0;

        //Monitor 'totalTicks', instead of the values of the encoders which are constantly reset.
        while(abs(totalTicks) < tickGoal)
        {
            //Proportional algorithm to keep the robot going straight.
            motor[leftServo] = masterPower;
            motor[rightServo] = slavePower;

            error = SensorValue[leftEncoder] - SensorValue[rightEncoder];

            slavePower += error / kp;

            SensorValue[leftEncoder] = 0;
            SensorValue[rightEncoder] = 0;

            wait1Msec(100);

            //Add this iteration's encoder values to totalTicks.
            totalTicks+= SensorValue[leftEncoder];
        }
        motor[leftServo] = 0; // Stop the loop once the encoders have counted up the correct number of encoder ticks.
        motor[rightServo] = 0;
    }
    */
   public void mechmove (double myangle, double mypower, float myrot) {
       if (robot.FleftMotor != null && robot.BLeftMotor != null && robot.FrightMotor != null && robot.BRightMotor != null)
       {
           robot.FleftMotor.setPower(Range.clip(myrot + (mypower * ((Math.sin((myangle + 135) / 180 * 3.141592)))), -1, 1));
           robot.BLeftMotor.setPower(Range.clip(myrot + (mypower * ((Math.sin((myangle + 45) / 180 * 3.141592)))), -1, 1));
           robot.FrightMotor.setPower(Range.clip(-myrot + (mypower * ((Math.sin((myangle + 45) / 180 * 3.141592)))), -1, 1));
           robot.BRightMotor.setPower(Range.clip(-myrot + (mypower * ((Math.sin((myangle + 135) / 180 * 3.141592)))), -1, 1));
       }
   }
   public void moveEncoder(int rotations)
   {
    int in1 = robot.FrightMotor.getCurrentPosition();
       int in2 = robot.FleftMotor.getCurrentPosition();
       int in4 = robot.BRightMotor.getCurrentPosition();
       int in3 = robot.BLeftMotor.getCurrentPosition();
    robot.FrightMotor.setTargetPosition(in1+rotations);
    robot.FleftMotor.setTargetPosition(in2+rotations);
    robot.BLeftMotor.setTargetPosition(in3+rotations);
    robot.BRightMotor.setTargetPosition(in4+rotations);

   }

    public void goBack(double speed, double period) {

        robot.BLeftMotor.setPower(-speed);
        robot.BRightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public void turnRight(double speed, double period) {

        //  Spin right x seconds
        robot.BLeftMotor.setPower(speed);
        robot.BRightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public void turnLeft(double speed, double period) {

        //  Spin Left for x seconds
        robot.BLeftMotor.setPower(-speed);
        robot.BRightMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            //  telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }

    }

    public void stopRobot() {
        robot.BLeftMotor.setPower(0.0);
        robot.BRightMotor.setPower(0.0);
        // robot.liftMotor.setPower(0.0);
        //robot.hWheel.setPower(0.0);
    }

    /* public void handUp() {

         robot.hand.setPosition(1.5);
     }

     public void handDown() {

         robot.hand.setPosition(0);
     }

     public void clawOpen() {

         robot.claw.setPosition(1);
     }
 */
    /*
    public void clawClose() {
        robot.claw.setPosition(0);
    }

    public void hLeft(double speed, double time) {
        robot.hWheel.setPower(-speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            // telemetry.update();
        }
    }

    public void hRight(double speed, double time) {
        robot.hWheel.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }
*/
   /*public void moveByRange(double speed,double distanceToWall) {

        while (opModeIsActive() && getDistance() > distanceToWall ) {
            //hLeft(0.8, 0.3);
            //stopRobot();
           // sleep(500);
            robot.hWheel.setPower(-speed);
        }

        while (opModeIsActive() && getDistance() < distanceToWall - 0.5) {
            //hRight(0.8, 0.3);
            //stopRobot();
            // sleep(500);
            robot.hWheel.setPower(speed);
        }

        stopRobot();
        sleep(200);
    }
*/

    public void liftUp(double speed, double time) {
        //robot.liftMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //  telemetry.update();
        }
    }


    public void liftDown(double speed, double time) {
        // robot.liftMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <= time)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            //   telemetry.update();
        }
    }

    public String vuforiaCapture() {
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeYAHIn/////AAAAGfVr1aFjUEHlh1uCvvWMJFtG8Y1D0YvNXpfCJTXkpgrNedm+jaqR+2trR9dGNzyeuHUMqo42P7DuJIp1IPDBDF5oepx6kw121V3vAc3sR5F43oix5brWapqdLcvFYcdFmWqg3AvIy436p1bkMhhJgcVEzXzIususTncxlVaHDDohnS9zN38qFcbFeKWH8cLG8lbt+2sNqoGJgOQ1/Oq6wEf3ceIS1x2BsguyUtkPLG0OQALkjbktRMdfLHe34ldDuCddP1ekNgkvwauoxOJqYKJKZX15h3VZfRtnp4mArn6Bxx8vWITXm690wfsdAio1LrRGm+NBovMapDxs9IKJuiH53nEoYrvat8IGG9IhMp67";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        int i = 0;
        RelicRecoveryVuMark vuMark = null;
        while (i < 2) {
            i++;
            telemetry.addData("i", "%s", i);
            sleep(500);
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        return vuMark.toString();
    }

    public void initialize() {
        robot.init(hardwareMap);
        // calibrateGyro();
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);

        //robot.claw.setPosition(.5);
        // robot.liftMotor.setPower(ARM_UP_POWER);

        sleep(2000);

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    public String colorSenseRev() {

        String color = "NOT READ";
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (time.time() < 5) {
            if (robot.colorSensorRev.red() > robot.colorSensorRev.blue() + 10) {
                telemetry.addData("color rev", "RED");
                telemetry.update();
                color = "RED";
                break;
            } else if (robot.colorSensorRev.blue() > robot.colorSensorRev.red() + 3) {
                telemetry.addData("color rev", "BLUE");
                telemetry.update();
                color = "BLUE";
                break;
            } else {
                telemetry.addData("color rev", "UNKNOWN");
                telemetry.update();
                color = "UNKNOWN";
            }
        }
        return color;
    }

    /*  public String colorSense() {
          robot.colorSensor.enableLed(true);

          String color = "NOT READ";
          ElapsedTime time = new ElapsedTime();
          time.reset();

          while (time.time() < 5) {
              if (robot.colorSensor.red() > robot.colorSensor.blue() + 3) {
                  telemetry.addData("color", "RED");
                  telemetry.update();
                  color = "RED";
                  break;
              } else if (robot.colorSensor.blue() > robot.colorSensor.red() + 3) {
                  telemetry.addData("color", "BLUE");
                  telemetry.update();
                  color = "BLUE";
                  break;
              } else {
                  telemetry.addData("color", "UNKNOWN");
                  telemetry.update();
                  color = "UNKNOWN";
              }
          }

          robot.colorSensor.enableLed(false);
          return color;
      }
  */
    public void repositionBot(double angleDegrees) {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);
        telemetry.update();
        // sleep(2000);

        while (angles.firstAngle > angleDegrees || angles.firstAngle < -angleDegrees) {
            if (angles.firstAngle > angleDegrees) {
                turnRight(0.5, 0.1);

            } else if (angles.firstAngle < -angleDegrees) {
                turnLeft(0.5, 0.1);
            }

            stopRobot();
            sleep(100);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle > angleDegrees - 5 && angles.firstAngle < angleDegrees + 5) {
                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                sleep(1000);
                if(angles.firstAngle > angleDegrees -8 && angles.firstAngle < angleDegrees +8){
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    //   sleep(1000);
                    break;
                }

                telemetry.addData("heading", angles.firstAngle);
                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                // sleep(1000);
            }
        }}


    public void repositionBotAntiClock(double angleDegrees) {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData("firstAngle", angles.firstAngle);
        telemetry.update();
        //sleep(2000);

        while (angles.firstAngle < angleDegrees || angles.firstAngle > -angleDegrees) {
            if (angles.firstAngle < angleDegrees) {
                turnLeft(0.5, 0.1);

            } else if (angles.firstAngle > -angleDegrees) {
                turnRight(0.5, 0.1);
            }

            stopRobot();
            sleep(100);
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (angles.firstAngle < angleDegrees - 5 && angles.firstAngle > angleDegrees + 5) {
                if(angles.firstAngle > angleDegrees -5 && angles.firstAngle < angleDegrees +5){
                    telemetry.addData("firstAngle", angles.firstAngle);
                    telemetry.update();
                    //  sleep(1000);
                    break;
                }

                telemetry.addData("heading", angles.firstAngle);
                telemetry.addData("firstAngle", angles.firstAngle);
                telemetry.update();
                //  sleep(1000);
            }
        }

    }}


   /* public double getDistance(){

        double distance = robot.rangeSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
        // telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("inch", "%.2f inch", distance);
        telemetry.update();
      //  sleep(500);
        return distance;
    }
}
*/