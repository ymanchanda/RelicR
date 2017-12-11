package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="XtremeV Teleop", group="Team10515")
public class Team10515Teleop extends OpMode{

    /* Declare OpMode members. */
    Team10515HW robot       = new Team10515HW(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.025 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Say", "Ready to Play");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double hWheel;
        //double Arm;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        //right = -gamepad1.left_stick_y;
        //left = -gamepad1.right_stick_y;

        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;
        //hWheel = gamepad1.right_stick_x;
        //Arm = gamepad1.right_stick_y;

        // Combine drive and turn for blended motion.
        left  = (drive + turn)/2;
        right = (drive - turn)/2;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        robot.rightMotor.setPower(right);
        robot.leftMotor.setPower(left);

        if (gamepad1.dpad_left)
            robot.hWheel.setPower(0.4);
        else if (gamepad1.dpad_right)
            robot.hWheel.setPower(-0.4);
        else
            robot.hWheel.setPower(0.0);

        // Use gamepad X & b buttons to open and close the claw
       if (gamepad1.b){
            clawOffset = clawOffset + 0.010;
            if (clawOffset > 1) clawOffset = 1;
            robot.claw.setPosition(1 - clawOffset);
        } else if (gamepad1.x) {
            clawOffset = clawOffset - 0.010;
            if (clawOffset < 0) clawOffset = 0;
            robot.claw.setPosition(1 - clawOffset);
        }

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.a) {
            robot.liftMotor.setPower(robot.ARM_UP_POWER);
        }else if (gamepad1.y) {
            robot.liftMotor.setPower(robot.ARM_DOWN_POWER);
        }else {
           // robot.liftMotor.setPower(0.0);
        }

        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        //telemetry.addData("left",  "%.2f", left);
        //telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    //@Override
    public void stop()
    {

    }



}