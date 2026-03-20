;; Auto-generated. Do not edit!


(when (boundp 'hesai_ros_driver::LossPacket)
  (if (not (find-package "HESAI_ROS_DRIVER"))
    (make-package "HESAI_ROS_DRIVER"))
  (shadow 'LossPacket (find-package "HESAI_ROS_DRIVER")))
(unless (find-package "HESAI_ROS_DRIVER::LOSSPACKET")
  (make-package "HESAI_ROS_DRIVER::LOSSPACKET"))

(in-package "ROS")
;;//! \htmlinclude LossPacket.msg.html


(defclass hesai_ros_driver::LossPacket
  :super ros::object
  :slots (_total_packet_count _total_packet_loss_count ))

(defmethod hesai_ros_driver::LossPacket
  (:init
   (&key
    ((:total_packet_count __total_packet_count) 0)
    ((:total_packet_loss_count __total_packet_loss_count) 0)
    )
   (send-super :init)
   (setq _total_packet_count (round __total_packet_count))
   (setq _total_packet_loss_count (round __total_packet_loss_count))
   self)
  (:total_packet_count
   (&optional __total_packet_count)
   (if __total_packet_count (setq _total_packet_count __total_packet_count)) _total_packet_count)
  (:total_packet_loss_count
   (&optional __total_packet_loss_count)
   (if __total_packet_loss_count (setq _total_packet_loss_count __total_packet_loss_count)) _total_packet_loss_count)
  (:serialization-length
   ()
   (+
    ;; uint32 _total_packet_count
    4
    ;; uint32 _total_packet_loss_count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _total_packet_count
       (write-long _total_packet_count s)
     ;; uint32 _total_packet_loss_count
       (write-long _total_packet_loss_count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _total_packet_count
     (setq _total_packet_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _total_packet_loss_count
     (setq _total_packet_loss_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hesai_ros_driver::LossPacket :md5sum-) "363355020f4e7cc5a0f379abeda225bc")
(setf (get hesai_ros_driver::LossPacket :datatype-) "hesai_ros_driver/LossPacket")
(setf (get hesai_ros_driver::LossPacket :definition-)
      "uint32 total_packet_count
uint32 total_packet_loss_count
")



(provide :hesai_ros_driver/LossPacket "363355020f4e7cc5a0f379abeda225bc")


