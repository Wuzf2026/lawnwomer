;; Auto-generated. Do not edit!


(when (boundp 'hesai_ros_driver::Firetime)
  (if (not (find-package "HESAI_ROS_DRIVER"))
    (make-package "HESAI_ROS_DRIVER"))
  (shadow 'Firetime (find-package "HESAI_ROS_DRIVER")))
(unless (find-package "HESAI_ROS_DRIVER::FIRETIME")
  (make-package "HESAI_ROS_DRIVER::FIRETIME"))

(in-package "ROS")
;;//! \htmlinclude Firetime.msg.html


(defclass hesai_ros_driver::Firetime
  :super ros::object
  :slots (_data ))

(defmethod hesai_ros_driver::Firetime
  (:init
   (&key
    ((:data __data) (make-array 512 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64[512] _data
    (* 8    512)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[512] _data
     (dotimes (i 512)
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[512] _data
   (dotimes (i (length _data))
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get hesai_ros_driver::Firetime :md5sum-) "869425aed297bb57ab8d72e8d4c6bc7d")
(setf (get hesai_ros_driver::Firetime :datatype-) "hesai_ros_driver/Firetime")
(setf (get hesai_ros_driver::Firetime :definition-)
      "float64[512] data
")



(provide :hesai_ros_driver/Firetime "869425aed297bb57ab8d72e8d4c6bc7d")


