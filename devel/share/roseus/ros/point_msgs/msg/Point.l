;; Auto-generated. Do not edit!


(when (boundp 'point_msgs::Point)
  (if (not (find-package "POINT_MSGS"))
    (make-package "POINT_MSGS"))
  (shadow 'Point (find-package "POINT_MSGS")))
(unless (find-package "POINT_MSGS::POINT")
  (make-package "POINT_MSGS::POINT"))

(in-package "ROS")
;;//! \htmlinclude Point.msg.html


(defclass point_msgs::Point
  :super ros::object
  :slots (_data _vel ))

(defmethod point_msgs::Point
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) 0.0)
    )
   (send-super :init)
   (setq _data __data)
   (setq _vel (float __vel))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:serialization-length
   ()
   (+
    ;; float32[] _data
    (* 4    (length _data)) 4
    ;; float32 _vel
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _vel
     (setq _vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get point_msgs::Point :md5sum-) "f6f9f9c6cebad27e7295a45a8afd08e6")
(setf (get point_msgs::Point :datatype-) "point_msgs/Point")
(setf (get point_msgs::Point :definition-)
      "float32[] data
float32 vel
# geometry_msgs/Pose pose
# string name

")



(provide :point_msgs/Point "f6f9f9c6cebad27e7295a45a8afd08e6")


