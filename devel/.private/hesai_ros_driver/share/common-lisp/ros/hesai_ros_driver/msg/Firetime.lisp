; Auto-generated. Do not edit!


(cl:in-package hesai_ros_driver-msg)


;//! \htmlinclude Firetime.msg.html

(cl:defclass <Firetime> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 512 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass Firetime (<Firetime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Firetime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Firetime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hesai_ros_driver-msg:<Firetime> is deprecated: use hesai_ros_driver-msg:Firetime instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Firetime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hesai_ros_driver-msg:data-val is deprecated.  Use hesai_ros_driver-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Firetime>) ostream)
  "Serializes a message object of type '<Firetime>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Firetime>) istream)
  "Deserializes a message object of type '<Firetime>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 512))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 512)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Firetime>)))
  "Returns string type for a message object of type '<Firetime>"
  "hesai_ros_driver/Firetime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Firetime)))
  "Returns string type for a message object of type 'Firetime"
  "hesai_ros_driver/Firetime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Firetime>)))
  "Returns md5sum for a message object of type '<Firetime>"
  "869425aed297bb57ab8d72e8d4c6bc7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Firetime)))
  "Returns md5sum for a message object of type 'Firetime"
  "869425aed297bb57ab8d72e8d4c6bc7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Firetime>)))
  "Returns full string definition for message of type '<Firetime>"
  (cl:format cl:nil "float64[512] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Firetime)))
  "Returns full string definition for message of type 'Firetime"
  (cl:format cl:nil "float64[512] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Firetime>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Firetime>))
  "Converts a ROS message object to a list"
  (cl:list 'Firetime
    (cl:cons ':data (data msg))
))
