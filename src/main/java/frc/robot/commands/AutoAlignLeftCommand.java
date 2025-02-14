package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;
import frc.robot.tools.Limelight;

public class AutoAlignLeftCommand extends Command {

    private DriveSubsystem driveSubsystem;
    private EndEffectorSubsystem endEffectorSubsystem;
    private Limelight limelight;
    private boolean finished = false;

    public AutoAlignLeftCommand() {
        this.driveSubsystem = RobotContainer.driveSubsystem;
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
        this.limelight = RobotContainer.limelight;
        addRequirements(driveSubsystem);
        addRequirements(endEffectorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(limelight.hasTarget()) {
            double aprilTagLocation = LimelightHelpers.getTX(Constants.LimelightConstants.name);

            if(
                (aprilTagLocation ) < (-Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
                && (aprilTagLocation) > (-Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                // We are in the zone
                driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);
                endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);

                //SequentialCommandGroup asdf = new SequentialCommandGroup(null);
                //asdf.execute();

                //ParallelCommandGroup asdf = new ParallelCommandGroup(null);
                //asdf.execute();
            } else if(
                (aprilTagLocation) < (-Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                // Move left
                driveSubsystem.driveRobotRelative(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0.0);
            } else if (
                (aprilTagLocation) > (-Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                // Move right
                driveSubsystem.driveRobotRelative(0.0, -Constants.DriveConstants.kAutoAlignSpeed, 0.0);
            }
        }
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
