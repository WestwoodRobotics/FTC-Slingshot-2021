package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name = "test", group="TeleOp")
public class Teleop extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor cascadeMotor = null;

    @Override
    void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front");
        leftBack = hardwareMap.get(DcMotor.class, "left_Back");
        rightFront  = hardwareMap.get(DcMotor.class, "right_Front");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        cascadeMotor = hardwareMap.get(DcMotor.class, "cascade");

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
            double mag = VectorMath.magnitude(drive, turn);
            if(mag < 1){
                leftPower = VectorMath.clamp(drive + turn, -1.0, 1.0) ;
                rightPower = VectorMath.clamp(drive - turn, -1.0, 1.0) ;
            } else {
                leftPower = VectorMath.clamp(drive / mag + turn / mag, -1.0, 1.0);
                rightPower = VectorMath.clamp(drive / mag - turn / mag, -1.0, 1.0);
            }
            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            //Cascade
            double pull = -gamepad1.right_stick_y;
            cascadeMotor.setPower(VectorMath.clamp(pull));
            telemetry.addData("Encoder Value", cascadeMotor.getCurrentPosition());

            telemetry.update();
        }

    }
}