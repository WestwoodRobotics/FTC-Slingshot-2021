package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="kiwi")
public class KiwiOpMode extends OpMode {
    public DcMotorEx side1;
    public DcMotorEx side2;
    public DcMotorEx side3;

    @Override
    public void init() {
        side1 = hardwareMap.get(DcMotorEx.class, "side1");
        side2 = hardwareMap.get(DcMotorEx.class, "side2");
        side3 = hardwareMap.get(DcMotorEx.class, "side3");
        side1.setDirection(DcMotorSimple.Direction.REVERSE);
        side2.setDirection(DcMotorSimple.Direction.REVERSE);
        side3.setDirection(DcMotorSimple.Direction.FORWARD);
        side1.setVelocityPIDFCoefficients(15,0,0,5);
        side2.setVelocityPIDFCoefficients(15,0,0,5);
        side3.setVelocityPIDFCoefficients(15,0,0,5);

    }

    @Override
    public void loop() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        //This is negative because positive means its rotating clockwise and we want counter-clockwise
        double angularVelo = -gamepad1.right_stick_x;
        //yay
        telemetry.addData("turn: ", angularVelo);
        double timeSpent = time;
        double side1Velo = Math.sin(0)*y+Math.cos(0)*x+angularVelo;
        double side2Velo = Math.sin(120)*y-Math.cos(120)*x+angularVelo;
        double side3Velo = Math.sin(240)*y+Math.cos(240)*0.8*x+angularVelo;

        //scaling it down
        if(Math.abs(side1Velo)> Math.abs(side2Velo) && Math.abs(side1Velo)> Math.abs(side3Velo)){
            if(Math.abs(side1Velo) > 1){
                double divisor = Math.abs(side1Velo);
                side1Velo/= divisor;
                side2Velo /= divisor;
                side3Velo /= divisor;
            }
        } else if(Math.abs(side2Velo)> Math.abs(side1Velo) && Math.abs(side2Velo)> Math.abs(side3Velo)){
            if(Math.abs(side2Velo) > 1){
                double divisor = Math.abs(side2Velo);
                side1Velo/= divisor;
                side2Velo /= divisor;
                side3Velo /= divisor;
            }
        } else{
            if(Math.abs(side3Velo) > 1){
                double divisor = Math.abs(side3Velo);
                side1Velo/= divisor;
                side2Velo /= divisor;
                side3Velo /= divisor;
            }
        }

        side1.setVelocity(side1Velo*2000);
        side2.setVelocity(side2Velo*2000);
        side3.setVelocity(side3Velo*2000);

    }
}
