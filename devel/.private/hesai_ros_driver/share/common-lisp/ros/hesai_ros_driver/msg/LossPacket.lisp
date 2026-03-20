; Auto-generated. Do not edit!


(cl:in-package hesai_ros_driver-msg)


;//! \htmlinclude LossPacket.msg.html

(cl:defclass <LossPacket> (roslisp-msg-protocol:ros-message)
  ((total_packet_count
    :reader total_packet_count
    :initarg :total_packet_count
    :type cl:integer
    :initform 0)
   (total_packet_loss_count
    :reader total_packet_loss_count
    :initarg :total_packet_loss_count
    :type cl:integer
    :initform 0))
)

(cl:defclass LossPacket (<LossPacket>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LossPacket>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LossPacket)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hesai_ros_driver-msg:<LossPacket> is deprecated: use hesai_ros_driver-msg:LossPacket instead.")))

(cl:ensure-generic-function 'total_packet_count-val :lambda-list '(m))
(cl:defmethod total_packet_count-val ((m <LossPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:total_packet_count-val is deprecated.  Use hesai_ros_driver-msg:total_packet_count instead.")
  (total_packet_count m))

(cl:ensure-generic-function 'total_packet_loss_count-val :lambda-list '(m))
(cl:defmethod total_packet_loss_count-val ((m <LossPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:total_packet_loss_count-val is deprecated.  Use hesai_ros_driver-msg:total_packet_loss_count instead.")
  (total_packet_loss_count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LossPacket>) ostream)
  "Serializes a message object of type '<LossPacket>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_packet_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_packet_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_packet_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_packet_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_packet_loss_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_packet_loss_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_packet_loss_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_packet_loss_count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LossPacket>) istream)
  "Deserializes a message object of type '<LossPacket>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_packet_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_packet_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_packet_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_packet_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'total_packet_loss_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'total_packet_loss_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'total_packet_loss_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'total_packet_loss_count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LossPacket>)))
  "Returns string type for a message object of type '<LossPacket>"
  "hesai_ros_driver/LossPacket")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LossPacket)))
  "Returns string type for a message object of type 'LossPacket"
  "hesai_ros_driver/LossPacket")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LossPacket>)))
  "Returns md5sum for a message object of type '<LossPacket>"
  "363355020f4e7cc5a0f379abeda225bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LossPacket)))
  "Returns md5sum for a message object of type 'LossPacket"
  "363355020f4e7cc5a0f379abeda225bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LossPacket>)))
  "Returns full string definition for message of type '<LossPacket>"
  (cl:format cl:nil "uint32 total_packet_count~%uint32 total_packet_loss_count~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LossPacket)))
  "Returns full string definition for message of type 'LossPacket"
  (cl:format cl:nil "uint32 total_packet_count~%uint32 total_packet_loss_count~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LossPacket>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LossPacket>))
  "Converts a ROS message object to a list"
  (cl:list 'LossPacket
    (cl:cons ':total_packet_count (total_packet_count msg))
    (cl:cons ':total_packet_loss_count (total_packet_loss_count msg))
))
