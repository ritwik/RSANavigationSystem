
(cl:in-package :asdf)

(defsystem "beaconfinder-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Beacon" :depends-on ("_package_Beacon"))
    (:file "_package_Beacon" :depends-on ("_package"))
    (:file "Beacons" :depends-on ("_package_Beacons"))
    (:file "_package_Beacons" :depends-on ("_package"))
  ))