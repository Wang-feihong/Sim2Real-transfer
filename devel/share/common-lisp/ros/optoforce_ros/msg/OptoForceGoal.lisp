; Auto-generated. Do not edit!


(cl:in-package optoforce_ros-msg)


;//! \htmlinclude OptoForceGoal.msg.html

(cl:defclass <OptoForceGoal> (roslisp-msg-protocol:ros-message)
  ((store
    :reader store
    :initarg :store
    :type cl:boolean
    :initform cl:nil)
   (acq_duration
    :reader acq_duration
    :initarg :acq_duration
    :type cl:float
    :initform 0.0)
   (publish_freq
    :reader publish_freq
    :initarg :publish_freq
    :type cl:integer
    :initform 0))
)

(cl:defclass OptoForceGoal (<OptoForceGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OptoForceGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OptoForceGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name optoforce_ros-msg:<OptoForceGoal> is deprecated: use optoforce_ros-msg:OptoForceGoal instead.")))

(cl:ensure-generic-function 'store-val :lambda-list '(m))
(cl:defmethod store-val ((m <OptoForceGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader optoforce_ros-msg:store-val is deprecated.  Use optoforce_ros-msg:store instead.")
  (store m))

(cl:ensure-generic-function 'acq_duration-val :lambda-list '(m))
(cl:defmethod acq_duration-val ((m <OptoForceGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader optoforce_ros-msg:acq_duration-val is deprecated.  Use optoforce_ros-msg:acq_duration instead.")
  (acq_duration m))

(cl:ensure-generic-function 'publish_freq-val :lambda-list '(m))
(cl:defmethod publish_freq-val ((m <OptoForceGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader optoforce_ros-msg:publish_freq-val is deprecated.  Use optoforce_ros-msg:publish_freq instead.")
  (publish_freq m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OptoForceGoal>) ostream)
  "Serializes a message object of type '<OptoForceGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'store) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acq_duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'publish_freq)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OptoForceGoal>) istream)
  "Deserializes a message object of type '<OptoForceGoal>"
    (cl:setf (cl:slot-value msg 'store) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acq_duration) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'publish_freq) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OptoForceGoal>)))
  "Returns string type for a message object of type '<OptoForceGoal>"
  "optoforce_ros/OptoForceGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OptoForceGoal)))
  "Returns string type for a message object of type 'OptoForceGoal"
  "optoforce_ros/OptoForceGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OptoForceGoal>)))
  "Returns md5sum for a message object of type '<OptoForceGoal>"
  "69e819ebc559709f7bffac7bbf22dba5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OptoForceGoal)))
  "Returns md5sum for a message object of type 'OptoForceGoal"
  "69e819ebc559709f7bffac7bbf22dba5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OptoForceGoal>)))
  "Returns full string definition for message of type '<OptoForceGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%bool store~%float64 acq_duration~%int32 publish_freq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OptoForceGoal)))
  "Returns full string definition for message of type 'OptoForceGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%bool store~%float64 acq_duration~%int32 publish_freq~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OptoForceGoal>))
  (cl:+ 0
     1
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OptoForceGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'OptoForceGoal
    (cl:cons ':store (store msg))
    (cl:cons ':acq_duration (acq_duration msg))
    (cl:cons ':publish_freq (publish_freq msg))
))
