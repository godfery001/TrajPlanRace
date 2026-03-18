;; Auto-generated. Do not edit!


(when (boundp 'cav_msgs::ObstacleVec)
  (if (not (find-package "CAV_MSGS"))
    (make-package "CAV_MSGS"))
  (shadow 'ObstacleVec (find-package "CAV_MSGS")))
(unless (find-package "CAV_MSGS::OBSTACLEVEC")
  (make-package "CAV_MSGS::OBSTACLEVEC"))

(in-package "ROS")
;;//! \htmlinclude ObstacleVec.msg.html


(defclass cav_msgs::ObstacleVec
  :super ros::object
  :slots (_timestamp _len _obstacleVec ))

(defmethod cav_msgs::ObstacleVec
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:len __len) 0)
    ((:obstacleVec __obstacleVec) ())
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _len (round __len))
   (setq _obstacleVec __obstacleVec)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:len
   (&optional __len)
   (if __len (setq _len __len)) _len)
  (:obstacleVec
   (&rest __obstacleVec)
   (if (keywordp (car __obstacleVec))
       (send* _obstacleVec __obstacleVec)
     (progn
       (if __obstacleVec (setq _obstacleVec (car __obstacleVec)))
       _obstacleVec)))
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; int32 _len
    4
    ;; cav_msgs/Obstacle[] _obstacleVec
    (apply #'+ (send-all _obstacleVec :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _len
       (write-long _len s)
     ;; cav_msgs/Obstacle[] _obstacleVec
     (write-long (length _obstacleVec) s)
     (dolist (elem _obstacleVec)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _len
     (setq _len (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; cav_msgs/Obstacle[] _obstacleVec
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _obstacleVec (let (r) (dotimes (i n) (push (instance cav_msgs::Obstacle :init) r)) r))
     (dolist (elem- _obstacleVec)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cav_msgs::ObstacleVec :md5sum-) "60d3a37403fa2a27da845f05fbc1cd50")
(setf (get cav_msgs::ObstacleVec :datatype-) "cav_msgs/ObstacleVec")
(setf (get cav_msgs::ObstacleVec :definition-)
      "
float64 	timestamp
int32   	len
Obstacle[]   	obstacleVec

================================================================================
MSG: cav_msgs/Obstacle
float64 	timestamp   #ros time
int32 		id
int16 		type        #refer to below definition       
float64     radius      # m, radius for object circle

float64 	x_global   #m, in the same coordinate system of the ego car
float64 	y_global   #m, in the same coordinate system of the ego car

float64 	speed_x    #not official definition

")



(provide :cav_msgs/ObstacleVec "60d3a37403fa2a27da845f05fbc1cd50")


