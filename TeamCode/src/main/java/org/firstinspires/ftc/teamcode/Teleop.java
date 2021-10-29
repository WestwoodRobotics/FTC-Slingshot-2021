package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import java.lang.Math;



@TeleOp(name = "test Differential", group="TeleOp")
public class Teleop extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront", new PIDCoefficients(1,1,1)),
            new CustomMotor("rightFront", new PIDCoefficients(1,1,1)),
            new CustomMotor("leftBack", new PIDCoefficients(1,1,1)),
            new CustomMotor("rightBack", new PIDCoefficients(1,1,1)),
            new CustomMotor("cascadeMotor", new PIDCoefficients(1,1,1)),
            new CustomMotor("carouselMotor", null)
    }

    @Override
    void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor  = hardwareMap.get(DcMotor.class, "left_Front");
        motors[1].motor = hardwareMap.get(DcMotor.class, "right_Front");
        motors[2].motor  = hardwareMap.get(DcMotor.class, "left_Back");
        motors[3].motor = hardwareMap.get(DcMotor.class, "right_Back");
        motors[4].motor = hardwareMap.get(DcMotor.class, "cascade");
        motors[5].motor = hardwareMap.get(DcMotor.class, "car");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        cascadeMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        cascadeMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //drivetrain
            double leftPower;
            double rightPower;
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;

            leftPower = drive + turn;
            rightPower = drive - turn;

            if(Math.abs(leftPower)>1 || Math.abs(rightPower)>1){
                double greaterValue = (Math.abs(leftPower) > Math.abs(rightPower))? (leftPower):(rightPower);
                leftPower /= Math.abs(greaterValue);
                rightPower /= Math.abs(greaterValue);
            }


            motors[0].setPower(leftPower);
            motors[2].setPower(leftPower);
            motors[1].setPower(rightPower);
            motors[3].setPower(rightPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            //Cascade
            if(gamepad1.a & !gamepad1.b){
                motors[4].setPower(0.2);
            } else if(gamepad1.b & !gamepad1.b){
                motors[4].setPower(-0.2);
            } else{
                motors[4].setPower(0);
            }

            //Carousel
            if(gamepad1.left_bumper){
                motors[5].setPower(0.5);
            } else{
                motors[5].setPower(0.0);
            }

            telemetry.addData("Cascade Motor power: ", cascadeMotor.getPower());
            telemetry.update();
        }

    }
}