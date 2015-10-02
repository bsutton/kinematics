package robotics.arm;

import robotics.Pose;

public interface InvKinematics
{

	public void determine(Pose endEffectorPose);
}
