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
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor cascadeMotor = null;
    DcMotor carouselMotor = null;
    PIDCoefficients coeffs = new PIDCoefficients(1,1,1);
    PIDFController controller = new PIDFController(coeffs);


    @Override
    void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front");
        leftBack = hardwareMap.get(DcMotor.class, "left_Back");
        rightFront  = hardwareMap.get(DcMotor.class, "right_Front");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        cascadeMotor = hardwareMap.get(DcMotor.class, "cascade");
        carouselMotor = hardwareMap.get(DcMotor.class, "car");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        cascadeMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //drivetrain
            double leftPower;
            double rightPower;
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;

            if(Math.abs(drive)>1 || Math.abs(turn)>1){
                double greaterValue = (Math.abs(drive) >Math.abs(turn))? drive:turn;
                drive /= Math.abs(greaterValue);
                turn /= Math.abs(greaterValue);
            }
            leftPower = drive + turn;
            rightPower = drive - turn;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            //Cascade
            if(gamepad1.a & !gamepad1.b){
                cascadeMotor.setPower(0.2);
            } else if(gamepad1.b & !gamepad1.b){
                cascadeMotor.setPower(-0.2);
            } else{
                cascadeMotor.setPower(0);
            }

            //Carousel
            if(gamepad1.left_bumper){
                carouselMotor.setPower(0.5);
            } else{
                carouselMotor.setPower(0.0);
            }

            telemetry.addData("Cascade Motor power: ", cascadeMotor.getPower());
            telemetry.update();
        }

    }
}