package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Servo;


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
public class TestTeamHardwarePushbot

{
    boolean bLedOn = false;

    /* Public OpMode members. */
    public DcMotor  leftFrontMotor   = null;
    public DcMotor  rightFrontMotor  = null;

    //public DcMotor  liftMotor    = null;
    public DcMotor upMotor = null;
    public Servo    claw    = null;

    public Servo    hand   = null;

    ColorSensor colorSensor     = null;
    GyroSensor gyroSensor = null;
   // TouchSensor touchSensor     = null;

    static final String  LEFT_FRONT_MOTOR = "LeftFrontMotor";
    static final String  RIGHT_FRONT_MOTOR = "RightFrontMotor";

   // static final String  LIFT_MOTOR = "LiftMotor";
    static final String  UP_MOTOR = "UpMotor";
   static final String  CLAW = "Claw";
   static final String  HAND = "Hand";
    static final String  COLOR_SENSOR = "Color";
   static final String  GYRO_SENSOR = "Gyro";


   // public static final double MID_SERVO       =  0.5 ;
   public static final double ARM_UP_POWER    =  0.5 ;
    public static final double ARM_DOWN_POWER  = -0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    // bLedOn represents the state of the LED.
   // boolean bLedOn = false;

    /* Constructor */
    public TestTeamHardwarePushbot(){

    }
        public void add(int x, int y)
        {
           System.out.print(x + y) ;
        }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor   = hwMap.dcMotor.get(LEFT_FRONT_MOTOR);
        rightFrontMotor  = hwMap.dcMotor.get(RIGHT_FRONT_MOTOR);

        //liftMotor    = hwMap.dcMotor.get(LIFT_MOTOR);
       upMotor    = hwMap.dcMotor.get(UP_MOTOR);
        claw       = hwMap.servo.get(CLAW);
        hand       = hwMap.servo.get(HAND);
        colorSensor = hwMap.colorSensor.get(COLOR_SENSOR);
        gyroSensor = hwMap.gyroSensor.get(GYRO_SENSOR);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE


        //liftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
       upMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD
        claw.setDirection(Servo.Direction.REVERSE);
        hand.setDirection(Servo.Direction.FORWARD);

        // disable the  color sensor LED in the beginning

        colorSensor.enableLed(bLedOn);


        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);

        //liftMotor.setPower(0);
        upMotor.setPower(0);
        claw.setPosition(1);
        //hand.setPosition(.7);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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


