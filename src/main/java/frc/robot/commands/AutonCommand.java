package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

public class AutonCommand extends SequentialCommandGroup {

    public AutonCommand(
        SwerveSubsystem driveSubsystem//, Intake intake, Indexing indexing, Shooter shooter, Wrist wrist
    ) {
        addRequirements(driveSubsystem);
        
        final double fwdSpeed = 0.5; // idk
        // final double fwdTime = 2.0; // seconds
        // final double rotSpeed = 0.2; // radians
        // final double rotTime = 0.5; // seconds
        final double distance = 2.0;
        final double aimAngle = 60;

        addCommands(
            // Reset encoders
            // new InstantCommand(driveSubsystem::resetEncoders),

            // new ParallelCommandGroup(
            //     Commands.runOnce(()->{
            //     intake.stop();
            //     indexing.stop();
            // })
            // )
            );
    }

}
