;; Auto-generated. Do not edit!


(when (boundp 'cav_msgs::Goal)
  (if (not (find-package "CAV_MSGS"))
    (make-package "CAV_MSGS"))
  (shadow 'Goal (find-package "CAV_MSGS")))
(unless (find-package "CAV_MSGS::GOAL")
  (make-package "CAV_MSGS::GOAL"))

(in-package "ROS")
;;//! \htmlinclude Goal.msg.html


(defclass cav_msgs::Goal
  :super ros::object
  :slots (_timestamp _Estop _Go _refPath ))

(defmethod cav_msgs::Goal
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:Estop __Estop) 0)
    ((:Go __Go) nil)
    ((:refPath __refPath) ())
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _Estop (round __Estop))
   (setq _Go __Go)
   (setq _refPath __refPath)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:Estop
   (&optional __Estop)
   (if __Estop (setq _Estop __Estop)) _Estop)
  (:Go
   (&optional (__Go :null))
   (if (not (eq __Go :null)) (setq _Go __Go)) _Go)
  (:refPath
   (&rest __refPath)
   (if (keywordp (car __refPath))
       (send* _refPath __refPath)
     (progn
       (if __refPath (setq _refPath (car __refPath)))
       _refPath)))
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; int16 _Estop
    2
    ;; bool _Go
    1
    ;; cav_msgs/RefPoint[] _refPath
    (apply #'+ (send-all _refPath :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int16 _Estop
       (write-word _Estop s)
     ;; bool _Go
       (if _Go (write-byte -1 s) (write-byte 0 s))
     ;; cav_msgs/RefPoint[] _refPath
     (write-long (length _refPath) s)
     (dolist (elem _refPath)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int16 _Estop
     (setq _Estop (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _Go
     (setq _Go (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; cav_msgs/RefPoint[] _refPath
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _refPath (let (r) (dotimes (i n) (push (instance cav_msgs::RefPoint :init) r)) r))
     (dolist (elem- _refPath)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cav_msgs::Goal :md5sum-) "5512e5dddfea9dde6ed8bf22e640c5e2")
(setf (get cav_msgs::Goal :datatype-) "cav_msgs/Goal")
(setf (get cav_msgs::Goal :definition-)
      "
##timestamp, ros time or utc time
float64     timestamp

#part 1: direct commands
int16 	Estop    # 0:none  1:low  2:medium   3:high
bool 	Go       # false, true

#part 2 -- lane center and drivable area
RefPoint[] refPath


================================================================================
MSG: cav_msgs/RefPoint
#reference path
float64 t   # time of this point
float64 x
float64 y
float64 v
float64 heading
float64 cr

#drivable area
float64 	left_width
float64 	right_width
")



(provide :cav_msgs/Goal "5512e5dddfea9dde6ed8bf22e640c5e2")


