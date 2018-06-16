package jaci.pathfinder;

import Team4450.Lib.Util;

/**
 * The Trajectory object contains an array of Segments that represent the location, velocity, acceleration, jerk and heading
 * of a particular point in the trajectory.
 *
 * Trajectories can be generated with the Pathfinder class
 *
 * @author Jaci
 */
public class Trajectory 
{
	private String	name = "";

	public Trajectory( String name )
	{
		this.setName(name);
	}
	
	/**
	 * @return the name
	 */
	public String getName()
	{
		return name;
	}

	/**
	 * @param name the name to set
	 */
	public void setName( String name )
	{
		this.name = name;
	}
	
    /**
     * The Trajectory Configuration outlines the rules to follow while generating the trajectory. This includes
     * the method used for 'fitting' the spline, the amount of samples to use, the time difference and maximum values
     * for the velocity, acceleration and jerk of the trajectory.
     */
    public static class Config 
    {

        public static final int SAMPLES_FAST = 1000;
        public static final int SAMPLES_LOW = SAMPLES_FAST * 10;
        public static final int SAMPLES_HIGH = SAMPLES_LOW * 10;

        public FitMethod fit;
        public int sample_count;
        public double dt, max_velocity, max_acceleration, max_jerk;

        /**
         * Create a Trajectory Configuration
         * @param fit                   The fit method to use
         * @param samples               How many samples to use to refine the path (higher = smoother, lower = faster)
         * @param dt                    The time delta between points (in seconds)
         * @param max_velocity          The maximum velocity the body is capable of travelling at (in meters per second)
         * @param max_acceleration      The maximum acceleration to use (in meters per second per second)
         * @param max_jerk              The maximum jerk (acceleration per second) to use
         */
        public Config(FitMethod fit, int samples, double dt, double max_velocity, double max_acceleration, double max_jerk) 
        {
            this.fit = fit;
            this.sample_count = samples;
            this.dt = dt;
            this.max_velocity = max_velocity;
            this.max_acceleration = max_acceleration;
            this.max_jerk = max_jerk;
        }
    }

    /**
     * A Trajectory Segment is a particular point in a trajectory. The segment contains the xy position and the velocity,
     * acceleration, jerk and heading at this point
     */
    public static class Segment 
    {
        public double dt, x, y, position, velocity, acceleration, jerk, heading;

        public Segment(double dt, double x, double y, double position, double velocity, double acceleration, double jerk, double heading) 
        {
            this.dt = dt;
            this.x = x;
            this.y = y;
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.jerk = jerk;
            this.heading = heading;
        }

        public Segment copy() 
        {
            return new Segment(dt, x, y, position, velocity, acceleration, jerk, heading);
        }

        public boolean equals(Segment seg) 
        {
            return  seg.dt == dt && seg.x == x && seg.y == y &&
                    seg.position == position && seg.velocity == velocity &&
                    seg.acceleration == acceleration && seg.jerk == jerk && seg.heading == heading;
        }

        public boolean fuzzyEquals(Segment seg) 
        {
            return  ae(seg.dt, dt) && ae(seg.x, x) && ae(seg.y, y) && ae(seg.position, position) &&
                    ae(seg.velocity, velocity) && ae(seg.acceleration, acceleration) && ae(seg.jerk, jerk) &&
                    ae(seg.heading, heading);
        }

        private boolean ae(double one, double two) 
        {
            return Math.abs(one - two) < 0.0001;        // Really small value
        }
    }

    /**
     * The Fit Method defines the function by which the trajectory path is generated. By default, the HERMITE_CUBIC method
     * is used.
     */
    public static enum FitMethod 
    {
        HERMITE_CUBIC, HERMITE_QUINTIC;
    }

    public Segment[] segments;

    public Trajectory(Segment[] segments) 
    {
        this.segments = segments;
    }

    public Trajectory(int length) 
    {
        this.segments = new Segment[length];
    }

    public Segment get(int index) 
    {
        return segments[index];
    }

    public int length()
    {
        return segments.length;
    }

    public Trajectory copy() 
    {
        Trajectory toCopy = new Trajectory(length());
        
        for (int i = 0; i < length(); i++) 
        {
            toCopy.segments[i] = get(i).copy();
        }
        
        return toCopy;
    }
    
    /**
     * Write a list of this trajectory's segments to the log file.
     */
    public void dumpToLog()
    {
    	Segment	seg;
    	
    	Util.consoleLog("%s segments:", name);
    	
    	for (int i = 0; i < length(); i++)
		{
			seg = get(i);
			Util.consoleLog("seg=%d x=%.2f y=%.2f pos=%.2f hdg=%.2f acc=%.2f", i, seg.x, seg.y, seg.position, 
					 Pathfinder.r2d(seg.heading), seg.acceleration);
		}
    }
}
