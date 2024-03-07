package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.auto.SpeakerAimbot;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto() {
        addRequirements();
        
        addCommands(
            new SpeakerAimbot().withTimeout(1),
            new ShootCommand(),
            RobotContainer.m_Swerb.followPathCommand("New Path").deadlineWith(new IntakeCommand()),
                // new SequentialCommandGroup(
                    // new FollowPathCommand(
                    //     // "AmpStartToNote3ToAmpShot"
                    //     "New Path"
                    //     )//,
                // )
            new SpeakerAimbot().withTimeout(1),
            new ShootCommand()

        );
    }

}
