; Auto-generated. Do not edit!


(cl:in-package hesai_ros_driver-msg)


;//! \htmlinclude UdpPacket.msg.html

(cl:defclass <UdpPacket> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0))
)

(cl:defclass UdpPacket (<UdpPacket>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UdpPacket>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UdpPacket)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hesai_ros_driver-msg:<UdpPacket> is deprecated: use hesai_ros_driver-msg:UdpPacket instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <UdpPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:stamp-val is deprecated.  Use hesai_ros_driver-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <UdpPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:data-val is deprecated.  Use hesai_ros_driver-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <UdpPacket>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:size-val is deprecated.  Use hesai_ros_driver-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UdpPacket>) ostream)
  "Serializes a message object of type '<UdpPacket>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UdpPacket>) istream)
  "Deserializes a message object of type '<UdpPacket>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UdpPacket>)))
  "Returns string type for a message object of type '<UdpPacket>"
  "hesai_ros_driver/UdpPacket")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UdpPacket)))
  "Returns string type for a message object of type 'UdpPacket"
  "hesai_ros_driver/UdpPacket")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UdpPacket>)))
  "Returns md5sum for a message object of type '<UdpPacket>"
  "c29f0f7365a75504f5f0008b5913cb94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UdpPacket)))
  "Returns md5sum for a message object of type 'UdpPacket"
  "c29f0f7365a75504f5f0008b5913cb94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UdpPacket>)))
  "Returns full string definition for message of type '<UdpPacket>"
  (cl:format cl:nil "# field		size(byte)~%# SOB 		2~%# angle		2~%# measure	5~%# block		SOB + angle + measure * 40~%# timestamp	4~%# factory	2~%# reserve	8~%# rpm		2~%# tail		timestamp + factory + reserve + rpm~%# packet	block * 6 + tail~%~%time stamp~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UdpPacket)))
  "Returns full string definition for message of type 'UdpPacket"
  (cl:format cl:nil "# field		size(byte)~%# SOB 		2~%# angle		2~%# measure	5~%# block		SOB + angle + measure * 40~%# timestamp	4~%# factory	2~%# reserve	8~%# rpm		2~%# tail		timestamp + factory + reserve + rpm~%# packet	block * 6 + tail~%~%time stamp~%uint8[] data~%uint32 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UdpPacket>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UdpPacket>))
  "Converts a ROS message object to a list"
  (cl:list 'UdpPacket
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':data (data msg))
    (cl:cons ':size (size msg))
))
