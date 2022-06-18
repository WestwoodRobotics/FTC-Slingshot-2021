package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;



@TeleOp(name = "Racoon Teleop",group="Iterative Opmode")

public class Teleop extends OpMode {
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx intakeMotor;

    @Override
    public void init(){
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.right_stick_x;
        double leftPower = y + x;
        double rightPower = y - x;
        double absLeft = Math.abs(leftPower);
        double absRight = Math.abs(rightPower);
        if(absLeft > absRight){
            if(absLeft > 1){
                leftPower/= absLeft;
                rightPower /= absLeft;
            }
        } else {
            if(absRight > 1){
                leftPower/= absRight;
                rightPower /= absRight;
            }
        }
        if(gamepad1.left_bumper){
            intakeMotor.setPower(1);
        } else if(gamepad1.right_bumper){
            intakeMotor.setPower(-1);
        } else{
            intakeMotor.setPower(0);
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }


}