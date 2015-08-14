#
# Robot PAU to motor mapping
# Copyright (C) 2014, 2015 Hanson Robotics
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA

import copy
import math
import NeckKinematics
import NeckVertical

class MapperBase:
  """
  Abstract base class for motor mappings.  Subclasses of MapperBase will
  be used to remap input values in received messages and convert them to
  motor units. Typically (but not always) the motor units are angles,
  measured in radians.

  The methods 'map' and '__init__' must be implemented when writing a
  new subclass.
  """

  def map(self, val):
    raise NotImplementedError

  def __init__(self, args, motor_entry):
    """
    On construction, mapper classes are passed an 'args' object and a
    'motor_entry' object, taken from a yaml file.  The args object will
    correspond to a 'function' property in the 'binding' stanza. The
    'motor_entry' will be the parent of 'binding', and is used mainly
    to reach 'min' and 'max' properties.  Thus, for example:

        foomotor:
           binding:
             function:
                - name: barmapper
                  bararg: 3.14
                  barmore:  2.718
           min: -1.0
           max: 1.0

    Thus, 'args' could be 'bararg', 'barmore', while 'motor_entry' would be
    the entire 'foomotor' stanza.
    """
    pass

# --------------------------------------------------------------

class Composite(MapperBase):
  """
  Composition mapper. This is initialized with an ordered list of
  mappers.  The output value is obtained by applying each of the
  mappers in sequence, one after the other.  The yaml file must
  specify a list of mappings in the function property.  For example:

          function:
            - name: quaternion2euler
              axis: z
            - name: linear
              scale: -2.92
              translate: 0

  would cause the quaternion2euler mapper function to be applied first,
  followed by the linear mapper.
  """
  def map(self, val):
    for mapper_instance in self.mapper_list:
      val = mapper_instance.map(val)
    return val

  def __init__(self, mapper_list):
    self.mapper_list = mapper_list

# --------------------------------------------------------------

class Linear(MapperBase):
  """
  Linear mapper.  This handles yaml entries that appear in one of the
  two forms.  One form uses inhomogenous coordinates, such as:

     function:
       - name: linear
         scale: -2.92
         translate: 0.3

  The other form specifies a min and max range, such as:

      function:
        - name: linear
          min: 0.252
          max: -0.342

  The first form just rescales the value to the indicated slope and
  offset. The second form rescales such that the indicated input min
  and max match the motor min and max.  For instance, in the second
  case, the input could be specified as min and max radians (for an
  angle), which is converted to motor min and max displacement.
  """

  def map(self, val):
    return (val + self.pretranslate) * self.scale + self.posttranslate

  def __init__(self, args, motor_entry):
    if args.has_key('scale') and args.has_key('translate'):
      self.pretranslate = 0
      self.scale = args['scale']
      self.posttranslate = args['translate']

    elif args.has_key('min') and args.has_key('max'):
      # Map the given 'min' and 'max' to the motor's 'min' and 'max'
      self.pretranslate = -args['min']
      self.scale = (motor_entry['max']-motor_entry['min'])/(args['max']-args['min'])
      self.posttranslate = motor_entry['min']

# --------------------------------------------------------------

class WeightedSum(MapperBase):
  """
  This will map min-max range for every term to intermediate min-max range
  (imin-imax) and then sum them up. Then the intermediate 0..1 range will be
  mapped to the motor min-max range.

  Example:

          function:
            - name: weightedsum
              imin: 0.402
              terms:
              - {min: 0, max: 1, imax: 0}
              - {min: 0, max: 0.6, imax: 1}
  """

  @staticmethod
  def _saturated(val, interval):
    return min(max(val, min(interval["min"],interval["max"])), max(interval["min"],interval["max"]))

  def map(self, vals):
    return sum(
      map(
        lambda (val, translate, scale, term):
          (self._saturated(val, term) + translate) * scale,
        zip(vals, self.pretranslations, self.scalefactors, self.termargs)
      )
    ) + self.posttranslate

  def __init__(self, args, motor_entry):
    range_motors = motor_entry["max"] - motor_entry["min"]

    self.posttranslate = args["imin"] * range_motors + motor_entry["min"]
    self.pretranslations = map(
      lambda term: -term["min"],
      args["terms"]
    )
    self.scalefactors = map(
      lambda term:
        (term["imax"]-args["imin"])/(term["max"]-term["min"])*range_motors,
      args["terms"]
    )
    self.termargs = args["terms"]

# --------------------------------------------------------------

class Quaternion2EulerYZX(MapperBase):

  def __init__(self, args, motor_entry):

    # These functions convert a quaternion to intrinsic Y-Z-X (or extrinsic
    # X-Z-Y) rotations in that order. So if X axis is the line of sight, Y -
    # vertical and Z - horizontal axes, Y, Z and X rotations will represent
    # Yaw, Pitch and Roll respectively. For a different rotation order, swap
    # q.x, q.y, q.z variables and 'x', 'y', 'z' keys.
    #
    # Properly speaking, these are not Euler angles, but Tate-Bryan angles.
    #
    # Note: Kudos to anyone who manages to put this swapping mechanism into
    # code, which also builds functions to compute only one of the rotations
    # like below. (it's harder than it looks)
    def tate_z(q) :
        z = math.asin(2 * (q.y * q.x + q.w * q.z))
        return z

    def tate_y(q) :
        y = math.atan2(
            -2 * (q.z * q.x - q.w * q.y),
            q.w**2 - q.y**2 - q.z**2 + q.x**2
          )
        # print "tate y=", y
        return y

    def tate_x(q) :
        x = math.atan2(
            -2 *(q.y * q.z - q.w * q.x),
            q.w**2 + q.y**2 - q.z**2 - q.x**2
          )
        # print "tate x=", x
        return x

    funcsByAxis = {
      'y': lambda q : tate_y(q),
      'z': lambda q : tate_z(q),
      'x': lambda q : tate_x(q)
    }
    self.map = funcsByAxis[args['axis'].lower()]

# --------------------------------------------------------------
#
# This is a small utility function that accepts a single quaternion from
# blender, and converts it to "animator sphere angles" (ASA).  These are
# actually Euler angles, but we give them the funny ASA name to avoid
# confusion with other euler-angle conventions.  The ASA convention used
# here is the same one documented in the neck_kinematics PDF. Its
# identical to the standard Euler angles used in undergraduate physics
# textbooks on rigid body motion (classical mechanics) but differs from
# the conventions used in ... other places (engineering, computer
# graphics).  That's OK; there are at least 24 different conventions.
#
# Blender provides us with quaternions in the coordinate frame:
# x-axis == body-left (Eva's left side)
# y-axis == straight backwards (pointing out of Eva's backside)
# z-axis == up
#
# We want to convert to Euler ngles with the following coordinates:
# x-axis == straight ahead
# y-axis == body-left
# z-axis == up
# The above is the textbook convention in undergraduate physics.
#
# We use the sphere-angle coordinates:
# theta == angle w.r.t. z-axis
# phi == azimuthal angle, from x axis
# psi == body roll
# that is,
# Rot = Rot(Z, phi) Rot (Y, theta) Rot (Z, psi)
#
# The formulas below are taken from Wikipedia, but modified so that
# they work for the spehre coordinates above.
# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
#
# Status: 1 July 2015: this now works exactly as expected! Woot!
def quat_to_asa(q) :

    # Sometimes someone sends us a null quaternion, which is bad.
    # Handle it gracefully.
    if q.w < 0.5 :
        e = q.x*q.x + q.y*q.y + q.z*q.z
        q.w = math.sqrt(1.0 - e)

    # print "Quaternions: %7f" %q.w, "%10.7f" % q.x, "%10.7f" % q.y, "%10.7f" % q.z
    # e = q.x*q.x + q.y*q.y + q.z*q.z
    # n = q.w*q.w + e
    # e = math.sqrt(e)
    # alpha = 2 * math.asin (e)
    # nex = q.x / e
    # ney = q.y / e
    # nez = q.z / e
    # print "alpha, axis:", alpha, nex, ney, nez
    #
    # To reconcile what blender is giving us, with the desired
    # coordinates, above, we make the following substitutions.
    #
    q_0 = q.w
    q_1 = -q.y
    q_2 = q.x
    q_3 = q.z
    #
    phi = math.atan2(
        (-q_0 * q_1 + q_2 * q_3),
        (q_0 * q_2 + q_1 * q_3)
      )
    costh = q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3
    if 1.0 < costh:
        costh = 1.0
    theta = math.acos(costh)
    psi = math.atan2(
        q_0 * q_1 + q_2 * q_3,
        - q_1 * q_3 + q_0 * q_2
      )
    #
    # print "Euler phi theta psi", phi, theta, psi
    return [phi, theta, psi]

# quat_fraction
# Given a quaternion 'q' and a fraction 'frac', return q times frac
# i.e. return a quaternion that is only a fraction of the total
# rotation given in q.
def quat_fraction(q, frac) :

    # print "Quaternions: %7f" %q.w, "%10.7f" % q.x, "%10.7f" % q.y, "%10.7f" % q.z
    e = q.x*q.x + q.y*q.y + q.z*q.z
    # one = q.w*q.w + e
    e = math.sqrt(e)
    # nex, ney, nez form a normalized unit vector.
    nex = q.x / e
    ney = q.y / e
    nez = q.z / e

    # alpha is the amount of rotation around the unit vector...
    alpha = 2 * math.asin (e)
    # print "alpha, axis:", alpha, nex, ney, nez
    # of which we take only a fraction...
    alpha *= frac
    e = math.sin(0.5 * alpha)
    fq = copy.deepcopy(q)
    fq.x = nex * e
    fq.y = ney * e
    fq.z = nez * e
    fq.w = math.sqrt(1.0 - e*e)

    return fq


# --------------------------------------------------------------
#
# This class accepts a single quaternion from blender, and converts
# it to motor angles for a single u-joint; by default, the upper-neck
# u-joint.  The blender quaternion coordinate system is described
# below.  The motor angles are computed using geometry appropriate
# for the Han neck mechanism; note that the Eva mechanism has different
# dimensions; these dimensions are currently hard-coded in
# NeckKinematics.py
#
class Quaternion2Upper(MapperBase):

  def __init__(self, args, motor_entry):

    self.hijoint = NeckKinematics.upper_neck()
    self.worst = 0

    # Returns the upper-neck left motor position, in radians
    def get_upper_left(q) :
        (phi, theta, psi) = quat_to_asa(q)
        self.hijoint.inverse_kinematics(theta, phi)
        # print "Motors: left right", self.hijoint.theta_l, self.hijoint.theta_r
        return self.hijoint.theta_l

    # Returns the upper-neck right motor position, in radians
    def get_upper_right(q) :
        (phi, theta, psi) = quat_to_asa(q)
        self.hijoint.inverse_kinematics(theta, phi)
        return self.hijoint.theta_r

    # Yaw (spin about neck-skull) axis, in radians, right hand rule.
    def get_yaw(q) :
        (phi, theta, psi) = quat_to_asa(q)

        # The twist factor is the actual correction for the fact
        # that the gimbal in the neck assembly prevents the u-joint
        # from rotating.
        twist = math.atan2 (math.tan(phi), math.cos(theta))
        if twist < 0 :
            twist += 3.14159265358979

        # The actual neck yaw.
        yaw = psi + twist
        if 3.1415926 < yaw:
            yaw -= 2 * 3.14159265358979

        # The bad_yaw is a "crude" approximation to the correct yaw ...
        # but in fact, the worst-case error is about half a percent,
        # and usually much much better (less than 1/20th of a percent)
        bad_yaw = psi + phi
        if 3.1415926 < bad_yaw:
            bad_yaw -= 2 * 3.14159265358979

        # Hacky corrections to bad quadrant of the arctan...
        if 2 < yaw-bad_yaw :
            yaw -= 3.14159265358979
        if yaw-bad_yaw < -2:
            yaw += 3.14159265358979

        if self.worst < abs(yaw-bad_yaw) :
            self.worst = abs(yaw-bad_yaw)
            print "worst: ", self.worst, self.worst * 180 / 3.14159

        if 0.01 < yaw-bad_yaw or yaw-bad_yaw < -0.01:
            print "Aieeeee! bad yaw!", yaw, bad_yaw, yaw-bad_yaw
            exit
        # print "Yaw: %9f" % bad_yaw, "%9.6f" % psi, "%9.6f" % phi,  \
        #      "%9.6f" % twist, "%10.7f" % yaw, "delta %10.7f" % (yaw-bad_yaw)
        # print "Yaw and approximation error", yaw, yaw-bad_yaw
        return yaw

    funcs = {
      'upleft': lambda q: get_upper_left(q),
      'upright': lambda q: get_upper_right(q),
      'yaw': lambda q: get_yaw(q)
    }
    self.map = funcs[args['axis'].lower()]

# --------------------------------------------------------------
#
# This class accepts a single quaternion from blender, and converts
# it to motor angles for two stacked u-joints; it splits up the
# quaternion into two parts, sending some of it to the upper and some
# to the lower u-joint.
#
class Quaternion2Split(MapperBase):

  def __init__(self, args, motor_entry):

    self.hijoint = NeckKinematics.upper_neck()
    self.lojoint = NeckKinematics.lower_neck()

    # split the angles in two, giving sume to the upper and some to the
    # lower ujoint. Valid values for split are -1.0 to 2.0, with a
    # 50%-50% given by setting split to 0.5.
    # Setting upper_split to 1.0 should result in exactly the same
    # behavior as the Quaternion2Upper class above.
    self.upper_split = 0.5
    self.lower_split = 1.0 - self.upper_split

    # XXX TODO remove hard-coded physical dimensions
    # Han neck mechanism has the upper joint being 8.93 centimeters
    # in front of the lower joint, and 112.16 centimeters above it.
    self.kappa = math.atan2(8.93, 112.16)

    # Returns the upper-neck left motor position, in radians
    def get_upper_left(q) :
        fq = quat_fraction(q, self.upper_split)
        (phi, theta, psi) = quat_to_asa(fq)
        self.hijoint.inverse_kinematics(theta, phi)
        # print "Upper theta-phi:", theta, phi
        # print "Upper motors:", self.hijoint.theta_l, self.hijoint.theta_r
        return self.hijoint.theta_l

    # Returns the upper-neck right motor position, in radians
    def get_upper_right(q) :
        fq = quat_fraction(q, self.upper_split)
        (phi, theta, psi) = quat_to_asa(fq)
        self.hijoint.inverse_kinematics(theta, phi)
        return self.hijoint.theta_r

    # Returns the lower-neck left motor position, in radians
    def get_lower_left(q) :
        fq = quat_fraction(q, self.lower_split)
        (phi, theta, psi) = quat_to_asa(fq)
        # (phi, theta, eta) = NeckVertical.neck_cant(phi, theta, psi, self.kappa)
        self.lojoint.inverse_kinematics(theta, phi)
        # print "Lower theta-phi:", theta, phi
        # print "Lower motors:", self.lojoint.theta_l, self.lojoint.theta_r
        return self.lojoint.theta_l

    # Returns the lower-neck right motor position, in radians
    def get_lower_right(q) :
        fq = quat_fraction(q, self.lower_split)
        (phi, theta, psi) = quat_to_asa(fq)
        # (phi, theta, eta) = NeckVertical.neck_cant(phi, theta, psi, self.kappa)
        self.lojoint.inverse_kinematics(theta, phi)
        return self.lojoint.theta_r

    # Yaw (spin about neck-skull) axis, in radians, right hand rule.
    def get_yaw(q) :
        (phi, theta, psi) = quat_to_asa(q)

        # XXX TODO The neck cant produces a small correction to the yaw,
        # the formulas are there in neck_cant, but need to be integrated
        # here...

        # The byaw is a "crude" approximation to the correct yaw ...
        # but in fact, the worst-case error is about half a percent,
        # and usually much much better (less than 1/20th of a percent)
        byaw = psi + phi
        if 3.1415926 < byaw:
            byaw -= 2 * 3.14159265358979

        return byaw

    funcs = {
      'upleft':  lambda q: get_upper_left(q),
      'upright': lambda q: get_upper_right(q),
      'loleft':  lambda q: get_lower_left(q),
      'loright': lambda q: get_lower_right(q),
      'yaw': lambda q: get_yaw(q)
    }
    self.map = funcs[args['axis'].lower()]


# --------------------------------------------------------------
#
# This class accepts two diferent quaternions from blender, one for the
# upper neck joint, and one for the lower joint, and converts them to
# motor angles for the two stacked u-joints.  This is the "full
# function" neck, unlike the to classes above, which don't offer the
# full set of motions.
#
# XXX untested; blender is not currently generating two quaternions.
# Some fiddling to get the corrdinate systems all korrect might be
# needed.
#
class Quaternion2Dual(MapperBase):

  def __init__(self, args, motor_entry):

    self.hijoint = NeckKinematics.upper_neck()
    self.lojoint = NeckKinematics.lower_neck()

    # XXX TODO remove hard-coded physical dimensions
    # Han neck mechanism has the upper joint being 8.93 centimeters
    # in front of the lower joint, and 112.16 centimeters above it.
    self.kappa = math.atan2(8.93, 112.16)

    # Returns the upper-neck left motor position, in radians
    def get_upper_left(q) :
        (phi, theta, psi) = quat_to_asa(q)
        try:
            self.hijoint.inverse_kinematics(theta, phi)
        except OverflowError:
            print "Upper left motor jam", theta, phi
        # print "Upper motors:", self.hijoint.theta_l, self.hijoint.theta_r
        return self.hijoint.theta_l

    # Returns the upper-neck right motor position, in radians
    def get_upper_right(q) :
        (phi, theta, psi) = quat_to_asa(q)
        try:
            self.hijoint.inverse_kinematics(theta, phi)
        except OverflowError:
            print "Upper right motor jam", theta, phi
        return self.hijoint.theta_r

    # Returns the lower-neck left motor position, in radians
    def get_lower_left(q) :
        (phi, theta, psi) = quat_to_asa(q)
        # (phi, theta, eta) = NeckVertical.neck_cant(phi, theta, psi, self.kappa)
        try:
            self.lojoint.inverse_kinematics(theta, phi)
        except OverflowError:
            print "Lower left motor jam", theta, phi
        # print "Lower motors:", self.lojoint.theta_l, self.lojoint.theta_r
        return self.lojoint.theta_l

    # Returns the lower-neck right motor position, in radians
    def get_lower_right(q) :
        (phi, theta, psi) = quat_to_asa(q)
        # (phi, theta, eta) = NeckVertical.neck_cant(phi, theta, psi, self.kappa)
        # print "Lower theta-phi:", theta, phi
        try:
            self.lojoint.inverse_kinematics(theta, phi)
        except OverflowError:
            print "Lower right motor jam", theta, phi
        return self.lojoint.theta_r

    # Yaw (spin about neck-skull) axis, in radians, right hand rule.
    def get_yaw(q) :
        (phi, theta, psi) = quat_to_asa(q)

        # XXX TODO The neck cant produces a small correction to the yaw,
        # the formulas are there in neck_cant, but need to be integrated
        # here...

        # The byaw is a "crude" approximation to the correct yaw ...
        # but in fact, the worst-case error is about half a percent,
        # and usually much much better (less than 1/20th of a percent)
        byaw = psi + phi
        if 3.1415926 < byaw:
            byaw -= 2 * 3.14159265358979

        return byaw

    funcs = {
      'upleft':  lambda q: get_upper_left(q),
      'upright': lambda q: get_upper_right(q),
      'loleft':  lambda q: get_lower_left(q),
      'loright': lambda q: get_lower_right(q),
      'yaw': lambda q: get_yaw(q)
    }
    self.map = funcs[args['axis'].lower()]


_mapper_classes = {
  "linear": Linear,
  "weightedsum": WeightedSum,
  "quaternion2euler": Quaternion2EulerYZX,
  "quaternion2upper": Quaternion2Upper,
  "quaternion2split": Quaternion2Split,
  "quaternion2dual": Quaternion2Dual
}

def build(yamlobj, motor_entry):
  if isinstance(yamlobj, dict):
    return _mapper_classes[yamlobj["name"]](yamlobj, motor_entry)

  elif isinstance(yamlobj, list):
    return Composite(
      [build(func_entry, motor_entry) for func_entry in yamlobj]
    )
