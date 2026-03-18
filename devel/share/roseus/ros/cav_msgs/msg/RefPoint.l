;; Auto-generated. Do not edit!


(when (boundp 'cav_msgs::RefPoint)
  (if (not (find-package "CAV_MSGS"))
    (make-package "CAV_MSGS"))
  (shadow 'RefPoint (find-package "CAV_MSGS")))
(unless (find-package "CAV_MSGS::REFPOINT")
  (make-package "CAV_MSGS::REFPOINT"))

(in-package "ROS")
;;//! \htmlinclude RefPoint.msg.html


(defclass cav_msgs::RefPoint
  :super ros::object
  :slots (_t _x _y _v _heading _cr _left_width _right_width ))

(defmethod cav_msgs::RefPoint
  (:init
   (&key
    ((:t __t) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:v __v) 0.0)
    ((:heading __heading) 0.0)
    ((:cr __cr) 0.0)
    ((:left_width __left_width) 0.0)
    ((:right_width __right_width) 0.0)
    )
   (send-super :init)
   (setq _t (float __t))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _v (float __v))
   (setq _heading (float __heading))
   (setq _cr (float __cr))
   (setq _left_width (float __left_width))
   (setq _right_width (float __right_width))
   self)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:cr
   (&optional __cr)
   (if __cr (setq _cr __cr)) _cr)
  (:left_width
   (&optional __left_width)
   (if __left_width (setq _left_width __left_width)) _left_width)
  (:right_width
   (&optional __right_width)
   (if __right_width (setq _right_width __right_width)) _right_width)
  (:serialization-length
   ()
   (+
    ;; float64 _t
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _v
    8
    ;; float64 _heading
    8
    ;; float64 _cr
    8
    ;; float64 _left_width
    8
    ;; float64 _right_width
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _t
       (sys::poke _t (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _cr
       (sys::poke _cr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_width
       (sys::poke _left_width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_width
       (sys::poke _right_width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _t
     (setq _t (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _cr
     (setq _cr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_width
     (setq _left_width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_width
     (setq _right_width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get cav_msgs::RefPoint :md5sum-) "4578623091705a57837c7e08dd996a1f")
(setf (get cav_msgs::RefPoint :datatype-) "cav_msgs/RefPoint")
(setf (get cav_msgs::RefPoint :definition-)
      "#reference path
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



(provide :cav_msgs/RefPoint "4578623091705a57837c7e08dd996a1f")


