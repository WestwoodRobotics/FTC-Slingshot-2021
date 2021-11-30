package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "testAuton", group = "test")
public class TankAuton extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    IMUController imuController = null;
    DrivetrainMethods moving;

    double rotation = 0;
    CustomMotor[] motors = {
            new CustomMotor("leftFront"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightFront"),              //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("leftBack"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rightBack"),               //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("carousel"),                //new PIDCoefficients(15, 0, 1)),
            new CustomMotor("rotateArm"),                //new PIDCoefficients(15, 0, 1))
            new CustomMotor("intake")
    };

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        imuController = new IMUController(hardwareMap.get(BNO055IMU.class, "imu"));
        motors[0].motor = hardwareMap.get(DcMotorEx.class,"left Front");
        motors[1].motor = hardwareMap.get(DcMotorEx.class,"right Front");
        motors[2].motor = hardwareMap.get(DcMotorEx.class,"left Back");
        motors[3].motor = hardwareMap.get(DcMotorEx.class,"right Back");
        motors[4].motor = hardwareMap.get(DcMotorEx.class, "carousel");
        motors[5].motor = hardwareMap.get(DcMotorEx.class, "rotating Arm");
        motors[6].motor = hardwareMap.get(DcMotorEx.class, "intake");

        motors[0].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[2].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].motor.setDirection(DcMotorEx.Direction.FORWARD);
        motors[4].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[5].motor.setDirection(DcMotorEx.Direction.REVERSE);
        motors[6].motor.setDirection(DcMotorEx.Direction.FORWARD);

        motors[0].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[1].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[2].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[3].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[4].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[5].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[6].motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motors[0].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[4].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[5].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[6].motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[1].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[2].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[3].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[4].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[5].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);
        motors[6].motor.setVelocityPIDFCoefficients(15, 0, 0, 0);


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
