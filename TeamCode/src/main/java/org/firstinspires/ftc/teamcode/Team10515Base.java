package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public abstract class Team10515Base extends LinearOpMode {

    /* Declare OpMode members. */
    TestTeamHardwarePushbot  robot   = new TestTeamHardwarePushbot();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();



    public void goStraight(double speed, double period){

        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void goBack(double speed, double period){

        robot.leftFrontMotor.setPower(-speed);
        robot.rightFrontMotor.setPower(-speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void turnRight(double speed, double period){

        //  Spin right x seconds
        robot.leftFrontMotor.setPower(-speed);
        robot.rightFrontMotor.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }

    public void turnLeft(double speed, double period){

        //  Spin Left for x seconds
        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(-speed);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < period)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }

    public void stopRobot(){
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);

    }
    public void handUp(){
        robot.hand.setPosition(0);
    }
    public void handDown()
    {
    robot.hand.setPosition(.5);
    }
    public void clawOpen()
    {
        robot.claw.setPosition(1);
    }
    public void clawClose()
    {
        robot.claw.setPosition(0);
    }
}
