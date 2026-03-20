
(cl:in-package :asdf)

(defsystem "hesai_ros_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Firetime" :depends-on ("_package_Firetime"))
    (:file "_package_Firetime" :depends-on ("_package"))
    (:file "LossPacket" :depends-on ("_package_LossPacket"))
    (:file "_package_LossPacket" :depends-on ("_package"))
    (:file "Ptp" :depends-on ("_package_Ptp"))
    (:file "_package_Ptp" :depends-on ("_package"))
    (:file "UdpFrame" :depends-on ("_package_UdpFrame"))
    (:file "_package_UdpFrame" :depends-on ("_package"))
    (:file "UdpPacket" :depends-on ("_package_UdpPacket"))
    (:file "_package_UdpPacket" :depends-on ("_package"))
  ))