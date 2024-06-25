package frc.robot.SubSystems.SwerveSubSystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Vector2d;

public class Swerve extends SubsystemBase{
    
    public SwerveModule[] modules;
    private static Swerve m_swerveInstance;

    public Swerve(){

        modules = new SwerveModule[4];

        modules[0] = new SwerveModule(Consts.SwerveValues.TOP_RIGHT_DRIVING_MOTOR_ID, Consts.SwerveValues.TOP_RIGHT_STEERING_MOTOR_ID, Consts.SwerveValues.TOP_RIGHT_ENCODER_ID);
        modules[1] = new SwerveModule(Consts.SwerveValues.TOP_LEFT_DRIVING_MOTOR_ID, Consts.SwerveValues.TOP_LEFT_STEERING_MOTOR_ID, Consts.SwerveValues.TOP_LEFT_ENCODER_ID);
        modules[2] = new SwerveModule(Consts.SwerveValues.BOTTOM_LEFT_DRIVING_MOTOR_ID, Consts.SwerveValues.BOTTOM_LEFT_STEERING_MOTOR_ID, Consts.SwerveValues.BOTTOM_LEFT_ENCODER_ID);
        modules[3] = new SwerveModule(Consts.SwerveValues.BOTTOM_RIGHT_DRIVING_MOTOR_ID, Consts.SwerveValues.BOTTOM_RIGHT_STEERING_MOTOR_ID, Consts.SwerveValues.BOTTOM_RIGHT_ENCODER_ID);

    }

    public void driveWhileSpin(Vector2d driveVec, double spin){

        driveVec.rotateBy(Math.toRadians(90));

        Vector2d[] finalModuleVecs = new Vector2d[4];

        for(int i = 0; i < 4; i++){
            Vector2d tempNormlisedVec = new Vector2d(Consts.SwerveValues.ROBOT_CONSTS_VECTOR[i]);
            tempNormlisedVec.normalise();

            Vector2d tempSpinVec = new Vector2d(tempNormlisedVec.mul(spin));
            Vector2d tempDriveVec = new Vector2d(driveVec).mul(Consts.SwerveValues.MAX_DRIVE_SPEED);

            finalModuleVecs[i] = new Vector2d(tempSpinVec.add(tempDriveVec));
            modules[i].vectorToModule(finalModuleVecs[i]);
        } 
    }

    public static Swerve getInstance() {
        if (m_swerveInstance == null) {
            m_swerveInstance = new Swerve();
        }
        return m_swerveInstance;
    }
}
