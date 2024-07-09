package frc.robot.Utils;

public interface Consts {

    public class SwerveValues{
        
        public static final int TOP_RIGHT_DRIVING_MOTOR_ID = 17;
        public static final int TOP_RIGHT_STEERING_MOTOR_ID = 13;
        public static final int TOP_RIGHT_ENCODER_ID = 5;

        public static final int TOP_LEFT_DRIVING_MOTOR_ID = 18;
        public static final int TOP_LEFT_STEERING_MOTOR_ID = 62;
        public static final int TOP_LEFT_ENCODER_ID = 4;

        public static final int BOTTOM_LEFT_DRIVING_MOTOR_ID = 3;
        public static final int BOTTOM_LEFT_STEERING_MOTOR_ID = 2;
        public static final int BOTTOM_LEFT_ENCODER_ID = 0;

        public static final int BOTTOM_RIGHT_DRIVING_MOTOR_ID = 8;
        public static final int BOTTOM_RIGHT_STEERING_MOTOR_ID = 27;
        public static final int BOTTOM_RIGHT_ENCODER_ID = 1;

        public static final double DRIVING_MOTOR_KP = 0.05 * 2;
        public static final double DRIVING_MOTOR_KI = 0.0005 * 2;
        public static final double DRIVING_MOTOR_KD = 0.075 * 2;
        public static final double DRIVING_MOTOR_FF = 0.025;
        

        public static final double STEERING_MOTOR_KP = 0.001;
        public static final double STEERING_MOTOR_KI = 0;
        public static final double STEERING_MOTOR_KD = 0;

        public static final double MAX_DRIVE_SPEED = 1;

        public static final double DRIVE_GEAR_RATION = 1 / 6.75;
        public static final double STEERING_GEAR_RATION = 1 / 12.8;
        public static final double WHEEL_PERIMETER = Math.PI * 0.095;

        public static final Vector2d[] ROBOT_CONSTS_VECTOR = {
            new Vector2d(1, -1), //top right
            new Vector2d(1, 1),//top left
            new Vector2d(-1, 1),//bottom left
            new Vector2d(-1, -1)//bottom right
        };

        public static final int CONTROLLER_PORT = 0;

        public static final double TOP_RIGHT_CAN_CODER_OFFSET = 0.293212890625;
        public static final double TOP_LEFT_CAN_CODER_OFFSET = 0.291748046875;        
        public static final double BOTTOM_LEFT_CAN_CODER_OFFSET = -0.1171875;
        public static final double BOTTOM_RIGHT_CAN_CODER_OFFSET = 0.310546875;

    }
} 
    