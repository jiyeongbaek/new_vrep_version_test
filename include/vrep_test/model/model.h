/*
dummy model
*/

#pragma once

namespace franka
{
	enum class Frame {
		kJoint1,
		kJoint2,
		kJoint3,
		kJoint4,
		kJoint5,
		kJoint6,
		kJoint7,
		kFlange,
		kEndEffector,
		kStiffness
	};

	/**
	 * Post-increments the given Frame by one.
	 *
	 * For example, Frame::kJoint2++ results in Frame::kJoint3.
	 *
	 * @param[in] frame Frame to increment.
	 *
	 * @return Original Frame.
	 */
	Frame operator++(Frame& frame, int /* dummy */) noexcept;


	class Model
	{
		/* dummy */
	};
	class RobotState
	{
		/* dummy */
	};
} 