; Auto-generated. Do not edit!


(cl:in-package cav_msgs-msg)


;//! \htmlinclude PlanedPath.msg.html

(cl:defclass <PlanedPath> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0)
   (Go
    :reader Go
    :initarg :Go
    :type cl:boolean
    :initform cl:nil)
   (Estop
    :reader Estop
    :initarg :Estop
    :type cl:fixnum
    :initform 0)
   (forward_flag
    :reader forward_flag
    :initarg :forward_flag
    :type cl:boolean
    :initform cl:nil)
   (backward_flag
    :reader backward_flag
    :initarg :backward_flag
    :type cl:boolean
    :initform cl:nil)
   (planedPath
    :reader planedPath
    :initarg :planedPath
    :type (cl:vector cav_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'cav_msgs-msg:Point :initial-element (cl:make-instance 'cav_msgs-msg:Point))))
)

(cl:defclass PlanedPath (<PlanedPath>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanedPath>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanedPath)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_msgs-msg:<PlanedPath> is deprecated: use cav_msgs-msg:PlanedPath instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <PlanedPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:timestamp-val is deprecated.  Use cav_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'Go-val :lambda-list '(m))
(cl:defmethod Go-val ((m <PlanedPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:Go-val is deprecated.  Use cav_msgs-msg:Go instead.")
  (Go m))

(cl:ensure-generic-function 'Estop-val :lambda-list '(m))
(cl:defmethod Estop-val ((m <PlanedPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:Estop-val is deprecated.  Use cav_msgs-msg:Estop instead.")
  (Estop m))

(cl:ensure-generic-function 'forward_flag-val :lambda-list '(m))
(cl:defmethod forward_flag-val ((m <PlanedPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:forward_flag-val is deprecated.  Use cav_msgs-msg:forward_flag instead.")
  (forward_flag m))

(cl:ensure-generic-function 'backward_flag-val :lambda-list '(m))
(cl:defmethod backward_flag-val ((m <PlanedPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:backward_flag-val is deprecated.  Use cav_msgs-msg:backward_flag instead.")
  (backward_flag m))

(cl:ensure-generic-function 'planedPath-val :lambda-list '(m))
(cl:defmethod planedPath-val ((m <PlanedPath>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:planedPath-val is deprecated.  Use cav_msgs-msg:planedPath instead.")
  (planedPath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanedPath>) ostream)
  "Serializes a message object of type '<PlanedPath>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Go) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'Estop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'forward_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'backward_flag) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'planedPath))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'planedPath))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanedPath>) istream)
  "Deserializes a message object of type '<PlanedPath>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timestamp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'Go) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Estop) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'forward_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'backward_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'planedPath) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'planedPath)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cav_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanedPath>)))
  "Returns string type for a message object of type '<PlanedPath>"
  "cav_msgs/PlanedPath")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanedPath)))
  "Returns string type for a message object of type 'PlanedPath"
  "cav_msgs/PlanedPath")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanedPath>)))
  "Returns md5sum for a message object of type '<PlanedPath>"
  "c81ebe469d3c8540c20f292a56d467d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanedPath)))
  "Returns md5sum for a message object of type 'PlanedPath"
  "c81ebe469d3c8540c20f292a56d467d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanedPath>)))
  "Returns full string definition for message of type '<PlanedPath>"
  (cl:format cl:nil "~%float64 timestamp   # ROS time in seconds~%bool 	Go          # false, true~%int16   Estop       # 0:none  1:low  2:medium   3:high~%~%# move forward or backward, both of them should be set to ensure safety~%bool forward_flag       #true~%bool backward_flag      #false~%~%# note that all of the vectors should be set~%# t: must be 0.04 seconds per point, i.e., 25 points per second~%# x: degree, in global GNSS frame~%# y: degree, in global GNSS frame~%# v: vector of desired speed m/s~%# heading: vector of heading info, rad, north: pi/2, east:0~%# cr: curvature, note + or - for each element~%Point[] planedPath~%~%~%~%~%================================================================================~%MSG: cav_msgs/Point~%float64 t   # time of this point~%float64 x~%float64 y~%float64 v~%float64 heading~%float64 cr~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanedPath)))
  "Returns full string definition for message of type 'PlanedPath"
  (cl:format cl:nil "~%float64 timestamp   # ROS time in seconds~%bool 	Go          # false, true~%int16   Estop       # 0:none  1:low  2:medium   3:high~%~%# move forward or backward, both of them should be set to ensure safety~%bool forward_flag       #true~%bool backward_flag      #false~%~%# note that all of the vectors should be set~%# t: must be 0.04 seconds per point, i.e., 25 points per second~%# x: degree, in global GNSS frame~%# y: degree, in global GNSS frame~%# v: vector of desired speed m/s~%# heading: vector of heading info, rad, north: pi/2, east:0~%# cr: curvature, note + or - for each element~%Point[] planedPath~%~%~%~%~%================================================================================~%MSG: cav_msgs/Point~%float64 t   # time of this point~%float64 x~%float64 y~%float64 v~%float64 heading~%float64 cr~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanedPath>))
  (cl:+ 0
     8
     1
     2
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'planedPath) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanedPath>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanedPath
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':Go (Go msg))
    (cl:cons ':Estop (Estop msg))
    (cl:cons ':forward_flag (forward_flag msg))
    (cl:cons ':backward_flag (backward_flag msg))
    (cl:cons ':planedPath (planedPath msg))
))
