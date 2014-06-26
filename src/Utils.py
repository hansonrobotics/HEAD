from math import cos,sin

def euler2quaternion(yaw, pitch, roll):
  c1 = cos(yaw/2)
  c2 = cos(pitch/2)
  c3 = cos(roll/2)
  s1 = sin(yaw/2)
  s2 = sin(pitch/2)
  s3 = sin(roll/2)

  x = s1*s2*c3 + c1*c2*s3
  y = s1*c2*c3 + c1*s2*s3
  z = c1*s2*c3 - s1*c2*s3
  w = c1*c2*c3 - s1*s2*s3

  return (x, y, z, w)
