
(cl:in-package :asdf)

(defsystem "staubli_tx90_planning-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Coordinate_force" :depends-on ("_package_Coordinate_force"))
    (:file "_package_Coordinate_force" :depends-on ("_package"))
  ))