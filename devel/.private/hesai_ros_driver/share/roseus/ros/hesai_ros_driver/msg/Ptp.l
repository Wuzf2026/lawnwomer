;; Auto-generated. Do not edit!


(when (boundp 'hesai_ros_driver::Ptp)
  (if (not (find-package "HESAI_ROS_DRIVER"))
    (make-package "HESAI_ROS_DRIVER"))
  (shadow 'Ptp (find-package "HESAI_ROS_DRIVER")))
(unless (find-package "HESAI_ROS_DRIVER::PTP")
  (make-package "HESAI_ROS_DRIVER::PTP"))

(in-package "ROS")
;;//! \htmlinclude Ptp.msg.html


(defclass hesai_ros_driver::Ptp
  :super ros::object
  :slots (_ptp_lock_offset _ptp_status ))

(defmethod hesai_ros_driver::Ptp
  (:init
   (&key
    ((:ptp_lock_offset __ptp_lock_offset) 0)
    ((:ptp_status __ptp_status) (make-array 16 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _ptp_lock_offset (round __ptp_lock_offset))
   (setq _ptp_status __ptp_status)
   self)
  (:ptp_lock_offset
   (&optional __ptp_lock_offset)
   (if __ptp_lock_offset (setq _ptp_lock_offset __ptp_lock_offset)) _ptp_lock_offset)
  (:ptp_status
   (&optional __ptp_status)
   (if __ptp_status (setq _ptp_status __ptp_status)) _ptp_status)
  (:serialization-length
   ()
   (+
    ;; uint8 _ptp_lock_offset
    1
    ;; uint8[16] _ptp_status
    (* 1    16)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _ptp_lock_offset
       (write-byte _ptp_lock_offset s)
     ;; uint8[16] _ptp_status
     (princ _ptp_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _ptp_lock_offset
     (setq _ptp_lock_offset (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[16] _ptp_status
   (setq _ptp_status (make-array 16 :element-type :char))
   (replace _ptp_status buf :start2 ptr-) (incf ptr- 16)
   ;;
   self)
  )

(setf (get hesai_ros_driver::Ptp :md5sum-) "17a101cc667a4bf8eccaf77e6e093ba0")
(setf (get hesai_ros_driver::Ptp :datatype-) "hesai_ros_driver/Ptp")
(setf (get hesai_ros_driver::Ptp :definition-)
      "uint8 ptp_lock_offset
uint8[16] ptp_status
")



(provide :hesai_ros_driver/Ptp "17a101cc667a4bf8eccaf77e6e093ba0")


