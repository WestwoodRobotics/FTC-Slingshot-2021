package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import java.lang.Math;


public class CustomMotor {
    public String name;
    public DcMotorEx motor = null;
    public PIDCoefficients coeffs;
    public PIDFController controller;
    //NORMAL PID
    public CustomMotor(String Name, PIDCoefficients coefficients){
        name = Name;
        coeffs = coefficients;
        controller = new PIDFController(coeffs);
        controller.setInputBounds(-1.0, 1.0);
    }
    //NO CONTROL
    public CustomMotor(String Name) {
        name = Name;
    }
    //FEEDFORWARD CONTROL
    public CustomMotor(String Name, PIDCoefficients coefficients, double kv, double ka){
        name =Name;
        coeffs = coefficients;
        controller = new PIDFController(coeffs, kv, ka);
    }



    public double autonPower(double pos, double velocity, double acceleration){
        controller.setTargetPosition(pos);
        controller.setTargetVelocity(velocity);
        controller.setTargetAcceleration(acceleration);
        return controller.update(motor.getCurrentPosition());
    }
}