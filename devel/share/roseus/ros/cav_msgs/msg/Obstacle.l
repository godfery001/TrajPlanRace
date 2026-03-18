;; Auto-generated. Do not edit!


(when (boundp 'cav_msgs::Obstacle)
  (if (not (find-package "CAV_MSGS"))
    (make-package "CAV_MSGS"))
  (shadow 'Obstacle (find-package "CAV_MSGS")))
(unless (find-package "CAV_MSGS::OBSTACLE")
  (make-package "CAV_MSGS::OBSTACLE"))

(in-package "ROS")
;;//! \htmlinclude Obstacle.msg.html


(defclass cav_msgs::Obstacle
  :super ros::object
  :slots (_timestamp _id _type _radius _x_global _y_global _speed_x ))

(defmethod cav_msgs::Obstacle
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:id __id) 0)
    ((:type __type) 0)
    ((:radius __radius) 0.0)
    ((:x_global __x_global) 0.0)
    ((:y_global __y_global) 0.0)
    ((:speed_x __speed_x) 0.0)
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _radius (float __radius))
   (setq _x_global (float __x_global))
   (setq _y_global (float __y_global))
   (setq _speed_x (float __speed_x))
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:x_global
   (&optional __x_global)
   (if __x_global (setq _x_global __x_global)) _x_global)
  (:y_global
   (&optional __y_global)
   (if __y_global (setq _y_global __y_global)) _y_global)
  (:speed_x
   (&optional __speed_x)
   (if __speed_x (setq _speed_x __speed_x)) _speed_x)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; int32 _id
    4
    ;; int16 _type
    2
    ;; float64 _radius
    8
    ;; float64 _x_global
    8
    ;; float64 _y_global
    8
    ;; float64 _speed_x
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _id
       (write-long _id s)
     ;; int16 _type
       (write-word _type s)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x_global
       (sys::poke _x_global (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_global
       (sys::poke _y_global (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed_x
       (sys::poke _speed_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _type
     (setq _type (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x_global
     (setq _x_global (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_global
     (setq _y_global (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed_x
     (setq _speed_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cav_msgs::Obstacle :md5sum-) "a54a7e58941fdabf5915ce1f4cd97ed5")
(setf (get cav_msgs::Obstacle :datatype-) "cav_msgs/Obstacle")
(setf (get cav_msgs::Obstacle :definition-)
      "float64 	timestamp   #ros time
int32 		id
int16 		type        #refer to below definition       
float64     radius      # m, radius for object circle

float64 	x_global   #m, in the same coordinate system of the ego car
float64 	y_global   #m, in the same coordinate system of the ego car

float64 	speed_x    #not official definition

")



(provide :cav_msgs/Obstacle "a54a7e58941fdabf5915ce1f4cd97ed5")


