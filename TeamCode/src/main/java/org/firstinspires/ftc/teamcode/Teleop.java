package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "test", group="Differential")
public class Teleop extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = new DcMotor();
    DcMotor rightFront = new DcMotor();
    DcMotor leftBack = new DcMotor();
    DcMotor rightBack = new DcMotor();

}