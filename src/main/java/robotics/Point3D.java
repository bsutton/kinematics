package robotics;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Point3D
{

	private Vector3D point;
	private Frame frame;

	public Point3D(Frame frame, double x, double y, double z)
	{
		this.frame = frame;
		this.point = new Vector3D(x, y, z);
	}

	@Override 
	public String toString()
	{
		return getPoint().getX()+" "+getPoint().getY()+" "+getPoint().getZ();
	}

	public Frame getFrame()
	{
		return frame;
	}

	public Point3D(Vector3D point)
	{
		this.point = point;
	}

	public Point3D subtract(Transform transform)
	{
		return new Point3D(getPoint().subtract(transform.getVector()));
	}

	public Point3D rotate(Rotation rotation)
	{
		return new Point3D(rotation.applyTo(getPoint()));
	}

	public Point3D invRotate(Rotation rotation)
	{
		return new Point3D(rotation.applyInverseTo(getPoint()));
	}

	public Point3D add(Transform transform)
	{
		return transform.add(getPoint());
	}

	public double getX()
	{
		return getPoint().getX();
	}

	public double getY()
	{
		return getPoint().getY();
	}

	public double getZ()
	{
		return getPoint().getZ();
	}

	public Vector3D getPoint()
	{
		return point;
	}
}
