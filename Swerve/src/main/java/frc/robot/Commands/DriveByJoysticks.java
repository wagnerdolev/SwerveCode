package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubSystems.SwerveSubSystem.Swerve;
import frc.robot.Utils.Vector2d;

public class DriveByJoysticks extends Command{

    private Supplier<Double> m_leftJoystickX, m_leftJoystickY, m_rightJoystickX;
    private static DriveByJoysticks m_instance;

    public DriveByJoysticks(Supplier<Double> leftJoystickX, Supplier<Double> leftJoystickY, Supplier<Double> rightJoystickX){
        m_leftJoystickX = leftJoystickX;
        m_leftJoystickY = leftJoystickY;
        m_rightJoystickX = rightJoystickX;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        double leftJoystickX = m_leftJoystickX.get();
        double leftJoystickY = m_leftJoystickX.get();
        double rightJoystickX = m_leftJoystickX.get();

        Swerve.getInstance().driveWhileSpin(new Vector2d(leftJoystickX, leftJoystickY), rightJoystickX);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

}
