package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultDrive extends Command {

    // CommandJoystick joystick;
    CommandXboxController joystick;
    SwerveSubsystem m_Swerb;
    SlewRateLimiter xRateLimiter = new SlewRateLimiter(DriveConstants.kXYSlewRate);
    SlewRateLimiter yRateLimiter = new SlewRateLimiter(DriveConstants.kXYSlewRate);
    SlewRateLimiter wRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

    public DefaultDrive() {
        addRequirements(RobotContainer.m_Swerb);
        this.m_Swerb = RobotContainer.m_Swerb;
        // joystick = RobotContainer.commandJoystick;
        joystick = RobotContainer.driverXbox;

    }

    @Override
    public void execute() {
        double maxSpeed = Constants.DriveConstants.kMaxLinearSpeed;
        double vx = joystick.getRawAxis(0) * maxSpeed;
        double vy = joystick.getRawAxis(1) * maxSpeed;
        double vw = joystick.getRawAxis(4) * 0.1;
        
        // Apply slew rate limits
        vx = xRateLimiter.calculate(vx);
        vy = yRateLimiter.calculate(vy);
        vw = wRateLimiter.calculate(vw);

        SmartDashboard.putNumber("Vx", vx);
        SmartDashboard.putNumber("Vy", vy);
        SmartDashboard.putNumber("Vw", vw);

        //ChassisSpeeds speeds = new ChassisSpeeds(vy, vx, vw);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, vw, Rotation2d.fromDegrees(-m_Swerb.getYaw()));
        m_Swerb.drive(speeds);
    }

}
