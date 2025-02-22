// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools.parts;

import java.util.HashMap;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathBuilder {

	private static DriveSubsystem driveSubsystem;
	//private static HashMap<String, List<PathPlannerPath>> pathMap = new HashMap<>();

	public PathBuilder() {

		try {
			driveSubsystem = RobotContainer.driveSubsystem;

			RobotConfig config = RobotConfig.fromGUISettings();

			AutoBuilder.configure(
				driveSubsystem::getPoseEstimatorPose2d, 
				driveSubsystem::resetOdometry, 
				driveSubsystem::getChassisSpeedsRobotRelative,
				driveSubsystem::setChassisSpeedsRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards 
				new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                	    AutoConstants.PathPLannerConstants.kPPDriveConstants, // Translation PID constants
	                    AutoConstants.PathPLannerConstants.kPPTurnConstants // Rotation PID constants
            	),
				config, 
				() -> {
					// Boolean supplier that controls when the path will be mirrored for the red alliance
					// This will flip the path being followed to the red side of the field.
					// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
					var alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
				  		return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
			  	}, 
				driveSubsystem
			);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/*public List<PathPlannerPath> getPathCommand(String path) {
		return pathMap.get(path);
	}

	public void addPath(String pathName) {

		try {
			// Use the PathPlannerAuto class to get a path group from an auto
			pathMap.put(pathName, PathPlannerAuto.getPathGroupFromAutoFile(pathName));
		} catch (Exception e) {
			e.printStackTrace();
		}

		// You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
		//Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
	}*/
}
