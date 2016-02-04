geometry_msgs/Quaternion m_headRotation
geometry_msgs/Vector3    m_headTranslation
geometry_msgs/Quaternion m_neckRotation

float32 m_eyeGazeLeftPitch
float32 m_eyeGazeLeftYaw
float32 m_eyeGazeRightPitch
float32 m_eyeGazeRightYaw

#An array of blendshape coefficients.
#They describe an expression of a virtual face.
float32[] m_coeffs
# Then setting shapekey values, names must be passed
string[] m_shapekeys