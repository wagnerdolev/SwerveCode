package frc.robot.SubSystems.SwerveSubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class Swerve extends SubsystemBase{
    
    SwerveModule[] modules;

    public Swerve(){
        modules[0] = new SwerveModule(Consts.SwerveValues.TOP_RIGHT_DRIVING_MOTOR_ID, Consts.SwerveValues.TOP_RIGHT_STEERING_MOTOR_ID, Consts.SwerveValues.TOP_RIGHT_ENCODER_ID);
        modules[1] = new SwerveModule(Consts.SwerveValues.TOP_LEFT_DRIVING_MOTOR_ID, Consts.SwerveValues.TOP_LEFT_STEERING_MOTOR_ID, Consts.SwerveValues.TOP_LEFT_ENCODER_ID);
        modules[2] = new SwerveModule(Consts.SwerveValues.BOTTOM_LEFT_DRIVING_MOTOR_ID, Consts.SwerveValues.BOTTOM_LEFT_STEERING_MOTOR_ID, Consts.SwerveValues.BOTTOM_LEFT_ENCODER_ID);
        modules[3] = new SwerveModule(Consts.SwerveValues.BOTTOM_RIGHT_DRIVING_MOTOR_ID, Consts.SwerveValues.BOTTOM_RIGHT_STEERING_MOTOR_ID, Consts.SwerveValues.BOTTOM_RIGHT_ENCODER_ID);

    }

    public void driveWhileSpin(Vector2d driveVec, double spin){

        driveVec.rotateBy(Math.toRadians(90));

        Vector2d topRightVec = new Vector2d(1,-1).mul(spin).add(driveVec);
        Vector2d topLeftVec = new Vector2d(1,-1).mul(spin).add(driveVec);
        Vector2d bottomLeftVec = new Vector2d(1,-1).mul(spin).add(driveVec);
        Vector2d bottomRightVec = new Vector2d(1,-1).mul(spin).add(driveVec);

        this.modules[0].vectorToModule(topRightVec);
        this.modules[1].vectorToModule(topLeftVec);
        this.modules[2].vectorToModule(bottomLeftVec);
        this.modules[3].vectorToModule(bottomRightVec);

    }
}
