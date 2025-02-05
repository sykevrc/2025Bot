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
            double offset = LimelightHelpers.getTX(Constants.LimelightConstants.name);

            if((offset - Constants.kAutoAlignTolerance) > 0) {
                driveSubsystem.drive(0.3, 0, 0);
            } else {
                driveSubsystem.drive(-0.3, 0, 0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
