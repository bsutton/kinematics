package robotics.arm;

import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import robotics.Axis;
import robotics.Frame;
import robotics.Pose;

public abstract class ArmKinematics
{

	private InvKinematics invKinematics;

	/**
	 * Maps the name of a segment Definition to that Definition
	 */
	final private Map<Segment, Link> segments = new LinkedHashMap<>();

	private Frame frame;

	@SuppressWarnings("unused")
	private Pose pose;

	public static class Segment
	{
		String name;

		@Override
		public String toString()
		{
			return name;
		}
	}

	public ArmKinematics(Frame frame, Pose pose)
	{
		this.frame = frame;
		this.pose = pose;
	}

	/**
	 * The add(Link) and add (Joint) methods allow you to describe the physical
	 * geometry of your arm. Once the physical geometry of the arm is described
	 * you can use the 'setPosition' method to calculate the required joint
	 * angles for a given position.
	 * 
	 * @param link
	 * @throws DuplicateDefinition
	 */
	public Segment addLink(String name, double x, double y, double z,
			double roll, double pitch, double yaw)
	{
		Segment segment = new Segment();
		segment.name = name;
		segments.put(segment, new Link(name, x, y, z, roll, pitch, yaw));
		return segment;
	}

	public Segment addJoint(String name, Axis axis, double roll, double pitch,
			double yaw)
	{
		Segment segment = new Segment();
		segment.name = name;
		segments.put(segment, new Joint(name, axis, roll, pitch, yaw));
		return segment;
	}

	/**
	 * Returns a list of Links and joints up to and including the given Segment.
	 * 
	 * If the Segment is null then all Joints and Links are returned.
	 * 
	 * @param definition
	 * @return
	 */
	private List<Pose> getSegmentPoses(Segment segment)
	{
		List<Pose> results = new LinkedList<>();

		for (Entry<Segment, Link> def : segments.entrySet())
		{
			results.add(def.getValue());
			if (def.getKey() == segment)
				break;
		}
		return results;
	}

	/**
	 * set an implementation of the InvKinematics interface that knows how to
	 * compute the inverse kinematics for a particular robot arm
	 * 
	 * @param invKinematics
	 */
	public void setInvKinematics(InvKinematics invKinematics)
	{
		this.invKinematics = invKinematics;

	}

	public void resetJointsToZero()
	{
		for (Link segment : segments.values())
		{
			if (segment instanceof Joint)
			{
				((Joint) segment).setAngle(0.0);
			}
		}
	}

	/**
	 * 
	 * @param endEffectorLink
	 */
	public void setPosition(Pose endEffectorLink)
	{
		invKinematics.determine(endEffectorLink);

	}

	public Frame getFrame()
	{
		return frame;
	}

	/**
	 * determine the pose of "segment" in the frame of the arm
	 * 
	 * @param segment
	 * @return
	 */
	public Pose getSegmentPose(Segment segment)
	{

		Pose ret = new Pose(Vector3D.ZERO, new Rotation(RotationOrder.XYZ, 0.0,
				0.0, 0.0));
		for (Pose pose : getSegmentPoses(segment))
		{
			ret = ret.compound(pose);
		}
		return ret;
	}

	/**
	 * 
	 * @param segment
	 * @param angleRadians
	 */
	public void setJointAngle(Segment segment, double angleRadians)
	{
		accessJoint(segment).setAngle(angleRadians);
	}

	public Pose getEndEffectorPose()
	{
		return getSegmentPose(null);
	}

	/**
	 * 
	 * @param segment
	 * @return angle of the joint in radians
	 */
	public double getComputedJointAngle(Segment segment)
	{
		return accessJoint(segment).getSetAngle();
	}

	/**
	 * ensure that we dont try to access a Link as a Joint
	 * 
	 * @param segment
	 * @return
	 */
	private Joint accessJoint(Segment segment)
	{
		Link joint = segments.get(segment);
		if (!(joint instanceof Joint))
		{
			throw new RuntimeException(segment + " is not a joint");
		}
		return (Joint) joint;
	}

	public Link getLink(Segment segment)
	{
		Link link = segments.get(segment);
		if (link instanceof Joint)
		{
			throw new RuntimeException(segment + " is not a link");
		}
		return link;
	}

}
