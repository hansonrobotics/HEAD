from math import sin,cos,atan2,asin

class Quat():

  def toInYZX(self):
    """
    Produces intrinsic rotations of Y-Z-X axes in that order representing this
    quaternion.
    """
    (x, y, z, w) = self.params
    yrot = atan2(2*y*w-2*x*z, 1-2*y*y-2*z*z)
    zrot = asin(2*x*y + 2*z*w)
    xrot = atan2(2*x*w-2*y*z, 1-2*x*x-2*z*z)
    return (yrot, zrot, xrot)

  @classmethod
  def fromInYZX(cls, yrot, zrot, xrot):
    """
    Builds a quaternion from intrinsic rotations around Y-Z-X axes in that
    order.
    """
    c1 = cos(yrot/2.0)
    c2 = cos(zrot/2.0)
    c3 = cos(xrot/2.0)
    s1 = sin(yrot/2.0)
    s2 = sin(zrot/2.0)
    s3 = sin(xrot/2.0)

    x = s1*s2*c3 + c1*c2*s3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3
    w = c1*c2*c3 - s1*s2*s3

    return cls(x, y, z, w)

  def __repr__(self):
    return str(self.params)

  def __init__(self, x, y, z, w):
    self.params = (x, y, z, w)
