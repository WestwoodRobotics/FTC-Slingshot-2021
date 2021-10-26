package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name= "test Mecanum", group="TeleOp")
public class MecanumTeleop extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor cascadeMotor = null;

    @Override
    void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFront = hardwareMap.get(DcMotor.class, "left_Front");
        leftBack = hardwareMap.get(DcMotor.class, "left_Back");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        cascadeMotor = hardwareMap.get(DcMotor.class, "cascade");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        cascadeMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

    }
}