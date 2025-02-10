package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.tools.Limelight;

public class AutoAlignRightCommand extends Command{

    DriveSubsystem driveSubsystem;
    Limelight limelight;

    public AutoAlignRightCommand() {
        this.driveSubsystem = RobotContainer.driveSubsystem;
        this.limelight = RobotContainer.limelight;
        addRequirements(driveSubsystem);
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
                Math.abs(aprilTagLocation - Constants.DriveConstants.kAutoAlignOffset) < (Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
                && Math.abs(aprilTagLocation - Constants.DriveConstants.kAutoAlignOffset) > (Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);
            } else if(
                Math.abs(aprilTagLocation - Constants.DriveConstants.kAutoAlignOffset) < (Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                //driveSubsystem.drive(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0);
                driveSubsystem.driveRobotRelative(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0);
            } else if(Math.abs(aprilTagLocation - Constants.DriveConstants.kAutoAlignOffset) > (Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)) {
                //driveSubsystem.drive(0.0, -1 * Constants.DriveConstants.kAutoAlignSpeed, 0);
                driveSubsystem.driveRobotRelative(0.0, -Constants.DriveConstants.kAutoAlignSpeed, 0);
            }
        }

        driveSubsystem.driveRobotRelative(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
