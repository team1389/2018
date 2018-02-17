package com.team1389.autonomous;

import com.team1389.hardware.inputs.software.EncoderIn;
import com.team1389.hardware.value_types.Position;
import com.team1389.robot.RobotConstants;
import com.team1389.util.boolean_util.TimedBoolean;

public class Jerk
{
	private double x1, x2, x3, x4, v1, v2, v3, a1, a2, j, interval;
	
	public Jerk(double x1, double x2, double x3, double x4, double interval)
	{
		this.x1 = x1;
		this.x2 = x2;
		this.x3 = x3;
		this.x4 = x4;
		this.interval = interval;
		solve();
	}

	public static Jerk fromEncoder(EncoderIn<Position> encoder, double interval)
	{
		TimedBoolean timer1 = new TimedBoolean(interval),
					 timer2 = new TimedBoolean(interval),
					 timer3 = new TimedBoolean(interval);
		double x1 = toMeters(encoder.get());
		timer1.start();
		while(!timer1.get());
		double x2 = toMeters(encoder.get());
		timer2.start();
		while(!timer2.get());
		double x3 = toMeters(encoder.get());
		timer3.start();
		while(!timer3.get());
		double x4 = toMeters(encoder.get());
		return new Jerk(x1, x2, x3, x4, interval);
	}
	
	private static double toMeters(double ticks) {
		double rotations = ticks / 1024;
		double circumference = Math.PI * RobotConstants.WheelDiameter;
		return rotations * circumference;
	}

	private void solve()
	{
		/*
		 * 1) Calculate changes in velocity between x1, x2, x3 and x4 2) Divide
		 * the changes by interval to get v1, v2 and v3 3) Repeat for a1, a2 and
		 * j
		 */
		double v1 = (x2 - x1)/interval,
			   v2 = (x3 - x2)/interval,
			   v3 = (x4 - x3)/interval,
			   a1 = (v2 - v1)/interval,
			   a2 = (v3 - v2)/interval,
			   j  = (a2 - a1)/interval;
		this.v1 = v1;
		this.v2 = v2;
		this.v3 = v3;
		this.a1 = a1;
		this.a2 = a2;
		this.j  = j;
	}
	
	public double getJerk() {
		return j;
	}
	
	public double[] getAccelerations() {
		return new double[] {a1, a2};
	}
	
	public double[] getVelocities() {
		return new double[] {v1, v2, v3};
	}
	
	public double[] getPositions() {
		return new double[] {x1, x2, x3, x4};
	}
}
