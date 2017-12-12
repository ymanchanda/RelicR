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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public abstract class Team10515Base extends LinearOpMode {

    /* Declare OpMode members. */
    Team10515HW robot = new Team10515HW();   // Use our Team 10515 hardware
    ElapsedTime runtime = new ElapsedTime();

    public static final String TAG = "Vuforia VuMark Sample";
    public static final double ARM_DOWN_POWER  = 0.3;
    public static final double ARM_UP_POWER  = -0.3;


    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    public void goStraight(double speed, double period) {

        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void goBack(double speed, double period) {

        robot.leftMotor.setPower(-speed);
        robot.rightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void turnRight(double speed, double period) {

        //  Spin right x seconds
        robot.leftMotor.setPower(speed);
        robot.rightMotor.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void turnLeft(double speed, double period) {

        //  Spin Left for x seconds
        robot.leftMotor.setPower(-speed);
        robot.rightMotor.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }

    public void stopRobot() {
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);

        robot.hWheel.setPower(0.0);


    }

    public void handUp() {

        robot.hand.setPosition(1.0);
    }

    public void handDown() {

        robot.hand.setPosition(.1);
    }

    public void clawOpen() {

        robot.claw.setPosition(1);
    }

    public void clawClose() {
        robot.claw.setPosition(0);
    }

    public void hLeft(double speed, double time) {
        robot.hWheel.setPower(speed);
    }

    public void hRight(double speed, double time) {
        robot.hWheel.setPower(-speed);
    }

   /* public void calibrateGyro() {
        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().clear();
        telemetry.addData("Gyro Calibrating. Do Not Move %s","!");
        telemetry.update();



        if (robot.gyroSensor instanceof ModernRoboticsI2cGyro){
            telemetry.addData("modern robotics gyro","true");
            telemetry.update();
            sleep(3000);
        }else{
            telemetry.addData(" modern robotics gyro","false");
            telemetry.update();
            sleep(3000);
        }
        robot.gyroSensor.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && robot.gyroSensor.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        robot.gyroSensor.resetZAxisIntegrator();

        int rawX = robot.gyroSensor.rawX();
        int rawY = robot.gyroSensor.rawY();
        int rawZ = robot.gyroSensor.rawZ();
        int heading = robot.gyroSensor.getHeading();
        //int integratedZ = robot.gyroSensor.getIntegratedZValue();
        telemetry.clear();
        telemetry.addData("Values are","%s, %s, %s",rawX,rawY,heading);
        telemetry.update();

    }*/

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
        while (i < 3) {
            i++;
            telemetry.addData("i", "%s", i);
            sleep(2000);
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

        robot.claw.setPosition(1);
        robot.liftMotor.setPower(ARM_UP_POWER);

        sleep(2000);

        // Send telemetry message to signify robotrt waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

    }

    public String colorSense() {
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

    public void repositionBot() {

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (angles.firstAngle > 3.0 || angles.firstAngle < -3.0) {
            if (angles.firstAngle > 3.0) {

                turnRight(0.3, 0.2);


            } else if (angles.firstAngle < -3.0) {
                turnLeft(0.3, 0.2);

            }
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
    }
}
