import java.lang.Math;

public class VectorMath {
    public static double magnitude(double v1,double v2){
        return Math.sqrt(v1*v1+v2*v2);

    }
    public static double clamp(double value, double min, double max){
        if(value < min){
            return min;
        } else if(value > max){
            return max;
        }
        return value;
    }
}