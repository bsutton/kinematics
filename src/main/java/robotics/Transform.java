package robotics;

import java.text.NumberFormat;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

/**
 * A Transform allows you to define an 'offset' or 'movement' and apply it to an existing Point.
 * 
 * Transforms essentially implement vector arithmetic.
 * 
 *
 */
public class Transform
{
	/**
	 * The transform described as a vector from the origin of the 3D co-ordinate space. 
	 */
	private Vector3D transform;

	/**
	 * Create a transform which can be used to translate a point by the given x,y and z offsets.
	 * 
	 * @param xoffset
	 * @param yoffset
	 * @param zoffset
	 */
	Transform(double xoffset, double yoffset, double zoffset)
	{
		transform = new Vector3D(xoffset, yoffset, zoffset);

	}

	/**
	 * Create a transform based on the given Point.
	 * The transform is essentially a vector from the origin to the Point.
	 * 
	 * @param point the point used to describe the end point of the vector a start point of the origin is implied.
	 */
	public Transform(Point3D point)
	{
		double xoffset = point.getX();
		double yoffset = point.getY();
		double zoffset = point.getZ();
		transform = new Vector3D(xoffset, yoffset, zoffset);
	}

	/**
	 * Creates a transform which is the difference between the two given points calculated
	 * as endPoint - startPoint.
	 * 
	 * The start and end points essentially describe a vector which is used to create the transform.
	 * 
	 * @param endPoint the endPoint vector
	 * @param startPoint the starting point of the vector
	 */
	public Transform(Point3D endPoint, Point3D startPoint)
	{
		double xoffset = endPoint.getX() - startPoint.getX();
		double yoffset = endPoint.getY() - startPoint.getY();
		double zoffset = endPoint.getZ() - startPoint.getZ();

		transform = new Vector3D(xoffset, yoffset, zoffset);
	}


	public Transform(Vector3D ret)
	{
		transform = ret;
	}

	/**
	 * Applies this transform to given point.
	 * 
	 * Essentially vector maths is applied:
	 * pointToTransform + transform = returned point 
	 * 
	 * @param pointToTransform - the point that this transform is to be applied to.
	 * 
	 * @return the transformed Point.
	 */
	public Point3D add(Vector3D pointToTransform)
	{
		return new Point3D(pointToTransform.add(transform));
	}

	/**
	 * Subtracts the given point this transform.
	 * 
	 * Essentially vector maths is applied:
	 * transform - pointToTransform = returned point 
	 * 
	 * @param pointToTransform - the point that this transform is to be applied to.
	 * 
	 * @return the transformed Point.
	 */
	public Point3D subtract(Vector3D pointToTransform)
	{
		return new Point3D(transform.subtract(pointToTransform));
	}

	public void getRotation()
	{
		throw new RuntimeException("not yet implemented");

	}

	/**
	 * Returns the magnitude of the vector described by this transform from the origin 
	 * of the 3D co-ordinate space.
	 * 
	 * @return the magnitude or length of the transform as measured from the origin (0,0,0).
	 */
	public double getDistance()
	{

		return transform.distance(new Vector3D(0, 0, 0));

	}
	
	/**
	 * Returns a human readable form of the transform to 1 decimal place.
	 */
	@Override
	public String toString()
	{
		NumberFormat format = NumberFormat.getInstance();
		format.setMaximumFractionDigits(1);
		return format.format(transform.getX()) + " " + format.format(transform.getY()) + " "
				+ format.format(transform.getZ());
	}

	public Vector3D getVector()
	{
		return transform;
	}

}
