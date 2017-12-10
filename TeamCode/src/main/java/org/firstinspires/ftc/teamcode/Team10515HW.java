package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

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
public class Team10515HW
{
    /* Public OpMode members. */
    //public DcMotor  leftFrontMotor   = null;
    //public DcMotor  rightFrontMotor  = null;
    public DcMotor  rightMotor = null;
    public DcMotor  leftMotor = null;
    public DcMotor  hWheel = null;
    public DcMotor  liftMotor    = null;
    public Servo    hand   = null;
    public Servo    claw = null;

    ColorSensor colorSensor     = null;

    static final String  LEFT_MOTOR = "LMotor";
    static final String  RIGHT_MOTOR = "RMotor";
    static final String  H_WHEEL = "HWheel";
    static final String  LIFT_MOTOR = "LiftMotor";
    static final String  Claw = "Claw";
    static final String  Hand = "Hand";
    static final String  COLOR_SENSOR = "Color";


    public static final double ARM_UP_POWER    =  0.25 ;
    public static final double ARM_DOWN_POWER  = -0.25;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Team10515HW(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //leftFrontMotor   = hwMap.dcMotor.get(LEFT_FRONT_MOTOR);
        //rightFrontMotor  = hwMap.dcMotor.get(RIGHT_FRONT_MOTOR);
        leftMotor   = hwMap.dcMotor.get(LEFT_MOTOR);
        rightMotor  = hwMap.dcMotor.get(RIGHT_MOTOR);
        hWheel      = hwMap.dcMotor.get(H_WHEEL);
        liftMotor   = hwMap.dcMotor.get(LIFT_MOTOR);
        claw   = hwMap.servo.get(Claw);
        hand   = hwMap.servo.get(Hand);

        colorSensor = hwMap.colorSensor.get(COLOR_SENSOR);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection((DcMotor.Direction.FORWARD));
        hWheel.setDirection((DcMotor.Direction.REVERSE));
        liftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD

        colorSensor.enableLed(false);

        //claw.setDirection(Servo.Direction.REVERSE);
        //hand.setDirection(Servo.Direction.REVERSE);

        // Set all motors to zero power
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        hWheel.setPower(0);
        liftMotor.setPower(0);

        claw.setPosition(1);
        hand.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

