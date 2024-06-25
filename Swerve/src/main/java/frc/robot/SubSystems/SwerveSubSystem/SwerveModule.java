package frc.robot.SubSystems.SwerveSubSystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Consts;
import frc.robot.Utils.Funcs;
import frc.robot.Utils.Vector2d;

public class SwerveModule extends SubsystemBase{
    
    private CANSparkMax m_drivingMotor, m_steeringMotor;
    private CANcoder m_steeringEncoder;

    public SwerveModule(int drivingMotorID, int steeringMotorID, int CANcoderID){
        m_drivingMotor = new CANSparkMax(drivingMotorID, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
        m_steeringEncoder = new CANcoder(CANcoderID);

        m_drivingMotor.restoreFactoryDefaults();
        m_steeringMotor.restoreFactoryDefaults();

        m_drivingMotor.getPIDController().setP(Consts.SwerveValues.DRIVING_MOTOR_KP);
        m_drivingMotor.getPIDController().setI(Consts.SwerveValues.DRIVING_MOTOR_KI);
        m_drivingMotor.getPIDController().setD(Consts.SwerveValues.DRIVING_MOTOR_KD);
        m_drivingMotor.getPIDController().setFF(Consts.SwerveValues.DRIVING_MOTOR_FF);

        m_steeringMotor.getPIDController().setP(Consts.SwerveValues.STEERING_MOTOR_KP);
        m_steeringMotor.getPIDController().setI(Consts.SwerveValues.STEERING_MOTOR_KI);
        m_steeringMotor.getPIDController().setD(Consts.SwerveValues.STEERING_MOTOR_KD);

        m_steeringMotor.getEncoder().setPositionConversionFactor(Consts.SwerveValues.STEERING_GEAR_RATION * 360);
        m_drivingMotor.getEncoder().setVelocityConversionFactor(Consts.SwerveValues.DRIVE_GEAR_RATION * 60 * Consts.SwerveValues.WHEEL_PERIMETER);

    }
    public double getSteeringAngle(){
        return m_steeringMotor.getEncoder().getPosition();
    }

    public double getDrvingSpeed(){
        return m_drivingMotor.getEncoder().getVelocity();
    }
    

    public void vectorToModule(Vector2d v){
        v.rotateBy(Math.toRadians(90));
        double targetAngle = Math.toDegrees(v.theta());
        double currentAngle = m_steeringMotor.getEncoder().getPosition();
        double optimizedAngle = Funcs.closestAngle(currentAngle, targetAngle);
        double optimizedOppositeAngle = Funcs.closestAngle(currentAngle, targetAngle - 180);

        if (Math.abs(optimizedAngle) < Math.abs(optimizedOppositeAngle)){
            m_steeringMotor.getPIDController().setReference(currentAngle + optimizedAngle, ControlType.kPosition);  
            m_drivingMotor.getPIDController().setReference(v.mag(), ControlType.kVelocity);

        }

        else { 
            m_steeringMotor.getPIDController().setReference(currentAngle + optimizedOppositeAngle, ControlType.kPosition);  
            m_drivingMotor.getPIDController().setReference( -1 * v.mag(), ControlType.kVelocity);

        }
        
        m_drivingMotor.getPIDController().setReference(v.mag(), ControlType.kVelocity);
    }

    
}
