package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "soumil is bad at coding")
public class ClimberOp extends LinearOpMode {
    DcMotorEx staticHook;
    DcMotorEx liftMotor;
    double ticksPerRevS= 43.9*28;
    double ticksPerRevL = 37.9*28;
    @Override
    public void runOpMode(){
        staticHook = hardwareMap.get(DcMotorEx.class, "staticHook");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        staticHook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.right_stick_y != 0){
                liftMotor.setVelocity(10000* gamepad1.right_stick_y);
            } else{
                liftMotor.setVelocity(0);
            }
            if(gamepad1.left_stick_y != 0){
                staticHook.setVelocity(10000* gamepad1.left_stick_y);
            } else{
                liftMotor.setVelocity(0);
            }
            if(gamepad1.left_bumper){
                return;
            }
            telemetry.addData("liftMotorVelocity", liftMotor.getVelocity());
            telemetry.addData("staticHookVelocity",staticHook.getVelocity());
            telemetry.update();


        }

    }
}
