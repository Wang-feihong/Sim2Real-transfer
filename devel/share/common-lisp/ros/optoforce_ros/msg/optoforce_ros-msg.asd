
(cl:in-package :asdf)

(defsystem "optoforce_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "OptoForceAction" :depends-on ("_package_OptoForceAction"))
    (:file "_package_OptoForceAction" :depends-on ("_package"))
    (:file "OptoForceActionFeedback" :depends-on ("_package_OptoForceActionFeedback"))
    (:file "_package_OptoForceActionFeedback" :depends-on ("_package"))
    (:file "OptoForceActionGoal" :depends-on ("_package_OptoForceActionGoal"))
    (:file "_package_OptoForceActionGoal" :depends-on ("_package"))
    (:file "OptoForceActionResult" :depends-on ("_package_OptoForceActionResult"))
    (:file "_package_OptoForceActionResult" :depends-on ("_package"))
    (:file "OptoForceFeedback" :depends-on ("_package_OptoForceFeedback"))
    (:file "_package_OptoForceFeedback" :depends-on ("_package"))
    (:file "OptoForceGoal" :depends-on ("_package_OptoForceGoal"))
    (:file "_package_OptoForceGoal" :depends-on ("_package"))
    (:file "OptoForceResult" :depends-on ("_package_OptoForceResult"))
    (:file "_package_OptoForceResult" :depends-on ("_package"))
  ))