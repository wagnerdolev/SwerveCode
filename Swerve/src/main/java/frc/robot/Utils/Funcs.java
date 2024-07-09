package frc.robot.Utils;

public class Funcs {
    
    public static double modulo(double a, double b) {
        return ((a % b) + b) % b;
    }
    
    public static double closestAngle(double a, double b) {
        // get direction
        double dir = modulo(b, 360.0) - modulo(a, 360.0);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    public static double convertRotationsToDegrees(double rotations){
        //convert rotations to degrees
        rotations *= 360;
       
        //convert from -180 - 180 to 0 - 360 
        if(rotations < 0){
            rotations += 360;
        }
        return rotations;
    }

    public static double convertHalfCircleToFullCircle (double degrees){

    //convert from -180 - 180 to 0 - 360 
        if(degrees < 0){
            degrees += 360;
        }

        return degrees;
    }
}
