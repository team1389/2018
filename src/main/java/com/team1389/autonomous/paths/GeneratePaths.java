package com.team1389.autonomous.paths;

import com.team1389.robot.RobotConstants;
import com.team1389.robot.RobotSoftware;
import com.team1389.trajectory.PathFollowingSystem;
import com.team1389.trajectory.PathFollowingSystem.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class GeneratePaths
{
	PathFollowingSystem path;
	Constants constants;

	public GeneratePaths(RobotSoftware robo)
	{
		constants = new Constants(RobotConstants.MaxJerk, RobotConstants.MaxAcceleration, RobotConstants.MaxVelocity, 1,
				0, 0, robo.angle.get(), 2);
		path = new PathFollowingSystem(robo.drive.getAsTank(), robo.leftDriveT.getSensorPositionStream(),
				robo.rightDriveT.getSensorPositionStream(), robo.angle, constants);
	}

	public void generateDriveStraight()
	{
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
		Waypoint[] points = new Waypoint[] { new Waypoint(-4, -1, Pathfinder.d2r(-45)), new Waypoint(-2, -2, 0),
				new Waypoint(0, 0, 0) };

		Trajectory trajectory = Pathfinder.generate(points, config);

		// Wheelbase Width = 0.762m
		TankModifier modifier = new TankModifier(trajectory).modify(0.762);

		// Do something with the new Trajectories...
		//Trajectory left = modifier.getLeftTrajectory();
		//Trajectory right = modifier.getRightTrajectory();
		path.followPath(trajectory, false);
	}

	public void driveStraight()
	{
		generateDriveStraight();

	}

}
