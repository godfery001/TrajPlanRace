;; Auto-generated. Do not edit!


(when (boundp 'cav_msgs::PlanedPath)
  (if (not (find-package "CAV_MSGS"))
    (make-package "CAV_MSGS"))
  (shadow 'PlanedPath (find-package "CAV_MSGS")))
(unless (find-package "CAV_MSGS::PLANEDPATH")
  (make-package "CAV_MSGS::PLANEDPATH"))

(in-package "ROS")
;;//! \htmlinclude PlanedPath.msg.html


(defclass cav_msgs::PlanedPath
  :super ros::object
  :slots (_timestamp _Go _Estop _forward_flag _backward_flag _planedPath ))

(defmethod cav_msgs::PlanedPath
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:Go __Go) nil)
    ((:Estop __Estop) 0)
    ((:forward_flag __forward_flag) nil)
    ((:backward_flag __backward_flag) nil)
    ((:planedPath __planedPath) ())
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _Go __Go)
   (setq _Estop (round __Estop))
   (setq _forward_flag __forward_flag)
   (setq _backward_flag __backward_flag)
   (setq _planedPath __planedPath)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:Go
   (&optional (__Go :null))
   (if (not (eq __Go :null)) (setq _Go __Go)) _Go)
  (:Estop
   (&optional __Estop)
   (if __Estop (setq _Estop __Estop)) _Estop)
  (:forward_flag
   (&optional (__forward_flag :null))
   (if (not (eq __forward_flag :null)) (setq _forward_flag __forward_flag)) _forward_flag)
  (:backward_flag
   (&optional (__backward_flag :null))
   (if (not (eq __backward_flag :null)) (setq _backward_flag __backward_flag)) _backward_flag)
  (:planedPath
   (&rest __planedPath)
   (if (keywordp (car __planedPath))
       (send* _planedPath __planedPath)
     (progn
       (if __planedPath (setq _planedPath (car __planedPath)))
       _planedPath)))
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; bool _Go
    1
    ;; int16 _Estop
    2
    ;; bool _forward_flag
    1
    ;; bool _backward_flag
    1
    ;; cav_msgs/Point[] _planedPath
    (apply #'+ (send-all _planedPath :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _Go
       (if _Go (write-byte -1 s) (write-byte 0 s))
     ;; int16 _Estop
       (write-word _Estop s)
     ;; bool _forward_flag
       (if _forward_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _backward_flag
       (if _backward_flag (write-byte -1 s) (write-byte 0 s))
     ;; cav_msgs/Point[] _planedPath
     (write-long (length _planedPath) s)
     (dolist (elem _planedPath)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _Go
     (setq _Go (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int16 _Estop
     (setq _Estop (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _forward_flag
     (setq _forward_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _backward_flag
     (setq _backward_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; cav_msgs/Point[] _planedPath
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _planedPath (let (r) (dotimes (i n) (push (instance cav_msgs::Point :init) r)) r))
     (dolist (elem- _planedPath)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get cav_msgs::PlanedPath :md5sum-) "c81ebe469d3c8540c20f292a56d467d8")
(setf (get cav_msgs::PlanedPath :datatype-) "cav_msgs/PlanedPath")
(setf (get cav_msgs::PlanedPath :definition-)
      "
float64 timestamp   # ROS time in seconds
bool 	Go          # false, true
int16   Estop       # 0:none  1:low  2:medium   3:high

# move forward or backward, both of them should be set to ensure safety
bool forward_flag       #true
bool backward_flag      #false

# note that all of the vectors should be set
# t: must be 0.04 seconds per point, i.e., 25 points per second
# x: degree, in global GNSS frame
# y: degree, in global GNSS frame
# v: vector of desired speed m/s
# heading: vector of heading info, rad, north: pi/2, east:0
# cr: curvature, note + or - for each element
Point[] planedPath




================================================================================
MSG: cav_msgs/Point
float64 t   # time of this point
float64 x
float64 y
float64 v
float64 heading
float64 cr
")



(provide :cav_msgs/PlanedPath "c81ebe469d3c8540c20f292a56d467d8")


