package frc.robot.commands;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class DriveBackwardsCommand extends Command {

    private DriveSubsystem driveSubsystem;
    private boolean finished = false;

    public DriveBackwardsCommand() {
        addRequirements(driveSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TimerTask task = new TimerTask() {
            public void run() {
                System.out.println("DriveBackwardsCommand::initialize() - task finished");
                finished = true;
            }
        };

        Timer timer = new Timer("Timer");
        timer.schedule(task, 250);
        finished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveSubsystem.driveRobotRelative(Constants.DriveConstants.kAutoAlignSpeed, 0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
