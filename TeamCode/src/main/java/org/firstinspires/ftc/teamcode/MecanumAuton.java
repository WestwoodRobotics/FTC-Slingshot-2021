package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.lang.Math;
import java.util.Arrays;

@Autonomous(name = "TestAuton", group = "Auton")
public class MecanumAuton extends LinearOpMode {

    BNO055IMU imu;
    DistanceSensor front;
    DistanceSensor back;
    DistanceSensor left;
    DistanceSensor right:

    ElapsedTime runtime = new ElapsedTime();
    CustomMotor[] motors = {
            new CustomMotor("leftFront", new PIDCoefficients(15,0,1)),
            new CustomMotor("leftBack", new PIDCoefficients(15,0,1)),
            new CustomMotor("rightFront", new PIDCoefficients(15,0,1)),
            new CustomMotor("rightBack", new PIDCoefficients(15,0,1)),
            new CustomMotor("cascadeMotor", new PIDCoefficients(1,1,1)),
            new CustomMotor("carouselMotor", null)
    }

    Servo leftArm = null;
    Servo rightArm = null;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    @Override public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motors[0].motor  = hardwareMap.get(DcMotor.class, "left_Front");
        motors[1].motor = hardwareMap.get(DcMotor.class, "right_Front");
        motors[2].motor  = hardwareMap.get(DcMotor.class, "left_Back");
        motors[3].motor = hardwareMap.get(DcMotor.class, "right_Back");
        motors[4].motor = hardwareMap.get(DcMotor.class, "cascade");
        motors[5].motor = hardwareMap.get(DcMotor.class, "car");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

    }

}