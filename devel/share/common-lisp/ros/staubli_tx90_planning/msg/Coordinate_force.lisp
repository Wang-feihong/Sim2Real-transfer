; Auto-generated. Do not edit!


(cl:in-package staubli_tx90_planning-msg)


;//! \htmlinclude Coordinate_force.msg.html

(cl:defclass <Coordinate_force> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench)))
)

(cl:defclass Coordinate_force (<Coordinate_force>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Coordinate_force>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Coordinate_force)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name staubli_tx90_planning-msg:<Coordinate_force> is deprecated: use staubli_tx90_planning-msg:Coordinate_force instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Coordinate_force>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader staubli_tx90_planning-msg:header-val is deprecated.  Use staubli_tx90_planning-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <Coordinate_force>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader staubli_tx90_planning-msg:point-val is deprecated.  Use staubli_tx90_planning-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <Coordinate_force>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader staubli_tx90_planning-msg:wrench-val is deprecated.  Use staubli_tx90_planning-msg:wrench instead.")
  (wrench m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Coordinate_force>) ostream)
  "Serializes a message object of type '<Coordinate_force>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Coordinate_force>) istream)
  "Deserializes a message object of type '<Coordinate_force>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Coordinate_force>)))
  "Returns string type for a message object of type '<Coordinate_force>"
  "staubli_tx90_planning/Coordinate_force")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coordinate_force)))
  "Returns string type for a message object of type 'Coordinate_force"
  "staubli_tx90_planning/Coordinate_force")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Coordinate_force>)))
  "Returns md5sum for a message object of type '<Coordinate_force>"
  "3e225759865dd432aff4d4f1ea18a73b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Coordinate_force)))
  "Returns md5sum for a message object of type 'Coordinate_force"
  "3e225759865dd432aff4d4f1ea18a73b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Coordinate_force>)))
  "Returns full string definition for message of type '<Coordinate_force>"
  (cl:format cl:nil "Header header~%geometry_msgs/Point point~%geometry_msgs/Wrench wrench~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Coordinate_force)))
  "Returns full string definition for message of type 'Coordinate_force"
  (cl:format cl:nil "Header header~%geometry_msgs/Point point~%geometry_msgs/Wrench wrench~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Coordinate_force>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Coordinate_force>))
  "Converts a ROS message object to a list"
  (cl:list 'Coordinate_force
    (cl:cons ':header (header msg))
    (cl:cons ':point (point msg))
    (cl:cons ':wrench (wrench msg))
))
