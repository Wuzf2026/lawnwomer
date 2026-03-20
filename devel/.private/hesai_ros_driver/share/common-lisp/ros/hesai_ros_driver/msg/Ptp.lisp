; Auto-generated. Do not edit!


(cl:in-package hesai_ros_driver-msg)


;//! \htmlinclude Ptp.msg.html

(cl:defclass <Ptp> (roslisp-msg-protocol:ros-message)
  ((ptp_lock_offset
    :reader ptp_lock_offset
    :initarg :ptp_lock_offset
    :type cl:fixnum
    :initform 0)
   (ptp_status
    :reader ptp_status
    :initarg :ptp_status
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 16 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Ptp (<Ptp>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ptp>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ptp)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hesai_ros_driver-msg:<Ptp> is deprecated: use hesai_ros_driver-msg:Ptp instead.")))

(cl:ensure-generic-function 'ptp_lock_offset-val :lambda-list '(m))
(cl:defmethod ptp_lock_offset-val ((m <Ptp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:ptp_lock_offset-val is deprecated.  Use hesai_ros_driver-msg:ptp_lock_offset instead.")
  (ptp_lock_offset m))

(cl:ensure-generic-function 'ptp_status-val :lambda-list '(m))
(cl:defmethod ptp_status-val ((m <Ptp>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:ptp_status-val is deprecated.  Use hesai_ros_driver-msg:ptp_status instead.")
  (ptp_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ptp>) ostream)
  "Serializes a message object of type '<Ptp>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ptp_lock_offset)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'ptp_status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ptp>) istream)
  "Deserializes a message object of type '<Ptp>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ptp_lock_offset)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ptp_status) (cl:make-array 16))
  (cl:let ((vals (cl:slot-value msg 'ptp_status)))
    (cl:dotimes (i 16)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ptp>)))
  "Returns string type for a message object of type '<Ptp>"
  "hesai_ros_driver/Ptp")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ptp)))
  "Returns string type for a message object of type 'Ptp"
  "hesai_ros_driver/Ptp")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ptp>)))
  "Returns md5sum for a message object of type '<Ptp>"
  "17a101cc667a4bf8eccaf77e6e093ba0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ptp)))
  "Returns md5sum for a message object of type 'Ptp"
  "17a101cc667a4bf8eccaf77e6e093ba0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ptp>)))
  "Returns full string definition for message of type '<Ptp>"
  (cl:format cl:nil "uint8 ptp_lock_offset~%uint8[16] ptp_status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ptp)))
  "Returns full string definition for message of type 'Ptp"
  (cl:format cl:nil "uint8 ptp_lock_offset~%uint8[16] ptp_status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ptp>))
  (cl:+ 0
     1
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'ptp_status) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ptp>))
  "Converts a ROS message object to a list"
  (cl:list 'Ptp
    (cl:cons ':ptp_lock_offset (ptp_lock_offset msg))
    (cl:cons ':ptp_status (ptp_status msg))
))
