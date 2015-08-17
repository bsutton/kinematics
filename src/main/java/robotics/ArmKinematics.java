package robotics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public abstract class ArmKinematics
{

	private InvKinematics invKinematics;

	/**
	 * Maps the name of a segment Definition to that Definition
	 */
	private Map<String, Definition> definitions = new LinkedHashMap<>();

	private Frame frame;

	@SuppressWarnings("unused")
	private Pose pose;

	private Map<Definition, ComputationalPose> definitionPoses;

	public ArmKinematics(Frame frame, Pose pose)
	{
		this.frame = frame;
		this.pose = pose;
	}

	/**
	 * The add(LinkDefinition) and add (JointDefinition) methods allow you to
	 * describe the physical geometry of you arm. Once the physical geometry of
	 * the arm is described you can use the 'setPosition' method to move the arm
	 * to a given position.
	 * 
	 * @param link
	 * @throws DuplicateDefinition
	 */
	public void add(LinkDefinition link) throws DuplicateDefinition
	{
		if (definitionPoses != null)
			throw new IllegalStateException("You can't add new definitions after the DefinitionPoses have been created");

		if (definitions.containsKey(link.getName()))
			throw new DuplicateDefinition("The LinkDefinition " + link.getName() + "has already been added to the arm.");

		definitions.put(link.getName(), link);
	}

	@SuppressWarnings("unchecked")
	public <J extends iJoint>  J add(JointDefinition jointDef) throws DuplicateDefinition
	{
		if (definitionPoses != null)
			throw new IllegalStateException("You can't add new definitions after the DefinitionPoses have been created");

		if (definitions.containsKey(jointDef.getName()))
			throw new DuplicateDefinition("The JointDefinition " + jointDef.getName()
					+ "has already been added to the arm.");

		definitions.put(jointDef.getName(), jointDef);

		return (J) jointDef.createJoint();
	}

	/**
	 * Creates a list of Definition Poses for from the list of Definitions
	 * (links and joints) that have been added to the ArmKinematics.
	 * 
	 * @return
	 */
	private Map<Definition, ComputationalPose> createDefinitionPoses()
	{
		definitionPoses = new LinkedHashMap<>();

		for (Definition def : this.definitions.values())
		{
			definitionPoses.put(def, def.createPose());
		}
		return definitionPoses;
	}

	/**
	 * Returns a list of Poses for all of the definitions up to and including
	 * the given definition.
	 * 
	 * If the definition is null then all poses are returned.
	 * 
	 * @param definition
	 * @return
	 */
	public Map<Definition, ComputationalPose> getComputationalPoses(Definition definition)
	{
		Map<Definition, ComputationalPose> results = new LinkedHashMap<>(); 
		
		if (definitionPoses == null)
			createDefinitionPoses();

		for (Definition def : this.definitions.values())
		{
			results.put(def, this.definitionPoses.get(def));
			if (def == definition)
				break;
		}
		return results;
	}

	public void setInvKinematics(InvKinematics invKinematics)
	{
		this.invKinematics = invKinematics;

	}

	/**
	 * 
	 * @param endEffectorPose
	 */
	public void setPosition(Pose endEffectorPose)
	{
		invKinematics.determine(this, endEffectorPose);

	}

	public Frame getFrame()
	{
		return frame;
	}

	public Vector3D getPoint(JointDefinition jointDef)
	{
		Vector3D segmentPose = getSegmentPose(jointDef);
		return segmentPose;
	}

	private Vector3D getSegmentPose(Definition definition)
	{
		ArrayList<Pose> reversedSegments = new ArrayList<>();

		reversedSegments.addAll(getComputationalPoses(definition).values());
		Collections.reverse(reversedSegments);

		Vector3D ret = Vector3D.ZERO;

		for (Pose segment : reversedSegments)
		{
			// System.out.println("'" + segment.getKey() + "' "
			// + segment.getValue());
			ret = ret.add(segment.getTransform().getVector());

			ret = segment.getRotation().applyInverseTo(ret);
			// System.out.println(ret);
		}
		return ret;
	}

	public Vector3D getEndEffectorPose()
	{
		return getSegmentPose(null);
	}

	public JointDefinition getJointDefinition(String name)
	{
		return (JointDefinition) definitions.get(name);
	}

	/**
	 * Returns the Joint for the given Joint Definition.
	 * 
	 * @param definition
	 * @return
	 */
	public abstract iJoint getJoint(JointDefinition definition);

}
