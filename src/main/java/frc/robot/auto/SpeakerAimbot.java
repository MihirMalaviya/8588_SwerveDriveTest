package frc.robot.auto;

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

public class SpeakerAimbot extends Command {

    SwerveSubsystem m_Swerb;

    public SpeakerAimbot() {
        addRequirements(RobotContainer.m_Swerb);
    }

    @Override
    public void execute() {
        RobotContainer.m_Swerb.aimAtGoal();
    }
}
