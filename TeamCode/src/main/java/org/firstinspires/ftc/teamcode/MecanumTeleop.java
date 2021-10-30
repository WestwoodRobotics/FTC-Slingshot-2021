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
import java.util.Arrays;

@TeleOp(name= "test Mecanum", group="TeleOp")
public class MecanumTeleop extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront", new PIDCoefficients(1,1,1)),
            new CustomMotor("leftBack", new PIDCoefficients(1,1,1)),
            new CustomMotor("rightFront", new PIDCoefficients(1,1,1)),
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

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascadeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //drivetrain
            double leftPower;
            double rightPower;
            double y = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx = gamepad1.left_stick_x;

            double[] velocity = {
                    y + x - rx,
                    y - x +rx,
                    y - x - rx,
                    y + x - rx
            }
            double highestValue = 0;
            for(double x: velocity){
                if(Math.abs(x) > highestValue){
                    highestValue = Math.abs(x);
                }
            }
            if(highestValue > 1){
                for (double x: velocity){
                    x /= highestValue;
                }
            }

            for(int i = 0; i < 4; i++){
                motors[i].controller.setTargetPosition(velocity[i]);
                motors[i].motor.setPower(motors[i].controller.update(motors[i].motor.getCurrentPosition()));
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", array.toString(velocity));

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