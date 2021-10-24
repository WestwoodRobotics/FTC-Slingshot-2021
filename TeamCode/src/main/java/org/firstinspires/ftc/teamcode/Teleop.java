package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "test", group="Differential")
public class Teleop extends OpMode {
    ElapsedTime runtime = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

}