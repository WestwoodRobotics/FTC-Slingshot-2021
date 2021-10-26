import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import java.lang.Math;

public class CustomMotor {
    public String name;
    public DcMotor motor =null;
    public PIDCoefficients coeffs;
    public PIDFController controller;
    //NORMAL PID
    public CustomMotor(String Name, PIDCoefficients coefficients){
        name =Name;
        coeffs = coefficients;
        controller = new PIDFController(coeffs);
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

    public double power(double pos, double velocity, double acceleration){
        controller.setTargetPosition(pos);
        controller.setTargetVelocity(velocity);
        controller.setTargetAcceleration(acceleration);
        return controller.update(motor.getCurrentPosition());
    }
}