package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "testAuton", group = "test")
public class AutonController extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    IMUController imuController = null;
    DrivetrainMethods moving;

    double rotation = 0;
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),
            new CustomMotor("leftBack"),
            new CustomMotor("rightFront"),
            new CustomMotor("rightBack"),
            new CustomMotor("cascadeMotor")
    };


    Servo leftArm     = null;
    Servo rightArm    = null;
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        imuController = new IMUController(hardwareMap.get(BNO055IMU.class, "imu"));
        motors[0].motor             = hardwareMap.get(DcMotorEx.class,          "left Front");
        motors[1].motor             = hardwareMap.get(DcMotorEx.class,          "right Front");
        motors[2].motor             = hardwareMap.get(DcMotorEx.class,          "left Back");
        motors[3].motor             = hardwareMap.get(DcMotorEx.class,          "right Back");
        motors[4].motor             = hardwareMap.get(DcMotorEx.class,          "cascade");
        leftArm                     = hardwareMap.get(Servo.class,              "left Arm");
        rightArm                    = hardwareMap.get(Servo.class,              "right Arm");

        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[4].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[0].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[1].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[2].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[3].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[4].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        moving = new DrivetrainMethods(motors);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            rotation = imuController.deltaRotation();


            telemetry.addData("time", runtime.toString());
            telemetry.addData("change in rotation", rotation);
            telemetry.update();
            moving.move(0,3000,0,0, rotation);

        }



    }
}
