; Auto-generated. Do not edit!


(cl:in-package optoforce_ros-msg)


;//! \htmlinclude OptoForceResult.msg.html

(cl:defclass <OptoForceResult> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass OptoForceResult (<OptoForceResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OptoForceResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OptoForceResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name optoforce_ros-msg:<OptoForceResult> is deprecated: use optoforce_ros-msg:OptoForceResult instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <OptoForceResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader optoforce_ros-msg:result-val is deprecated.  Use optoforce_ros-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OptoForceResult>) ostream)
  "Serializes a message object of type '<OptoForceResult>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OptoForceResult>) istream)
  "Deserializes a message object of type '<OptoForceResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OptoForceResult>)))
  "Returns string type for a message object of type '<OptoForceResult>"
  "optoforce_ros/OptoForceResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OptoForceResult)))
  "Returns string type for a message object of type 'OptoForceResult"
  "optoforce_ros/OptoForceResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OptoForceResult>)))
  "Returns md5sum for a message object of type '<OptoForceResult>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OptoForceResult)))
  "Returns md5sum for a message object of type 'OptoForceResult"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OptoForceResult>)))
  "Returns full string definition for message of type '<OptoForceResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OptoForceResult)))
  "Returns full string definition for message of type 'OptoForceResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OptoForceResult>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OptoForceResult>))
  "Converts a ROS message object to a list"
  (cl:list 'OptoForceResult
    (cl:cons ':result (result msg))
))
