package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

/*
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
*/

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class TeamHardwarePushbot
{
    /* Public OpMode members. */
    //public DcMotor  leftFrontMotor   = null;
    //public DcMotor  rightFrontMotor  = null;
    public DcMotor  rightBackMotor = null;
    public DcMotor  leftBackMotor = null;
    public DcMotor  hWheel = null;
    public DcMotor  liftMotor    = null;
    public DcMotor upMotor = null;
   // public Servo    leftClaw    = null;
  //  public Servo    rightClaw   = null;
    public Servo claw = null;

   // ColorSensor colorSensor     = null;
   // TouchSensor touchSensor     = null;

    //static final String  LEFT_FRONT_MOTOR = "LeftFrontMotor";
    //static final String  RIGHT_FRONT_MOTOR = "RightFrontMotor";
   static final String  LEFT_BACK_MOTOR = "LeftBackMotor";
    static final String  RIGHT_BACK_MOTOR = "RightBackMotor";
    static final String  H_WHEEL = "HWheel";
    static final String  LIFT_MOTOR = "LiftMotor";
    static final String  UP_MOTOR = "UpMotor";
  // static final String  LEFT_CLAW = "LeftClaw";
   // static final String  RIGHT_CLAW = "RightClaw";
    static final String     Claw = "Claw";
   // static final String  COLOR_SENSOR = "sc";
   // static final String  TOUCH_SENSOR = "stouch";


   // public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.25 ;
    public static final double ARM_DOWN_POWER  = -0.25;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // bLedOn represents the state of the LED.
   // boolean bLedOn = false;

    /* Constructor */
    public TeamHardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //leftFrontMotor   = hwMap.dcMotor.get(LEFT_FRONT_MOTOR);
        //rightFrontMotor  = hwMap.dcMotor.get(RIGHT_FRONT_MOTOR);
        leftBackMotor = hwMap.dcMotor.get(LEFT_BACK_MOTOR);
        rightBackMotor = hwMap.dcMotor.get(RIGHT_BACK_MOTOR);
        hWheel = hwMap.dcMotor.get(H_WHEEL);
        liftMotor    = hwMap.dcMotor.get(LIFT_MOTOR);
        upMotor    = hwMap.dcMotor.get(UP_MOTOR);
        //leftClaw       = hwMap.servo.get(LEFT_CLAW);
        //rightClaw       = hwMap.servo.get(RIGHT_CLAW);
        claw   = hwMap.servo.get(Claw);

       // colorSensor = hwMap.colorSensor.get(COLOR_SENSOR);
//        touchSensor = hwMap.touchSensor.get(TOUCH_SENSOR);

        //leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD
        //rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection((DcMotor.Direction.REVERSE));
        hWheel.setDirection((DcMotor.Direction.REVERSE));
        liftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
       upMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
        //leftClaw.setDirection(Servo.Direction.REVERSE);
        //rightClaw.setDirection(Servo.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);

        // disable the  color sensor LED in the beginning
        //colorSensor.enableLed(bLedOn);

        // Set all motors to zero power
        //leftFrontMotor.setPower(0);
        //rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
        hWheel.setPower(0);
        liftMotor.setPower(0);
        upMotor.setPower(0);
        claw.setPosition(1);

        //leftClaw.setPosition(0);
        //rightClaw.setPosition(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

