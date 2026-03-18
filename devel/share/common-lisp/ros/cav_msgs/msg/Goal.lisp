; Auto-generated. Do not edit!


(cl:in-package cav_msgs-msg)


;//! \htmlinclude Goal.msg.html

(cl:defclass <Goal> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0)
   (Estop
    :reader Estop
    :initarg :Estop
    :type cl:fixnum
    :initform 0)
   (Go
    :reader Go
    :initarg :Go
    :type cl:boolean
    :initform cl:nil)
   (refPath
    :reader refPath
    :initarg :refPath
    :type (cl:vector cav_msgs-msg:RefPoint)
   :initform (cl:make-array 0 :element-type 'cav_msgs-msg:RefPoint :initial-element (cl:make-instance 'cav_msgs-msg:RefPoint))))
)

(cl:defclass Goal (<Goal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_msgs-msg:<Goal> is deprecated: use cav_msgs-msg:Goal instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:timestamp-val is deprecated.  Use cav_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'Estop-val :lambda-list '(m))
(cl:defmethod Estop-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:Estop-val is deprecated.  Use cav_msgs-msg:Estop instead.")
  (Estop m))

(cl:ensure-generic-function 'Go-val :lambda-list '(m))
(cl:defmethod Go-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:Go-val is deprecated.  Use cav_msgs-msg:Go instead.")
  (Go m))

(cl:ensure-generic-function 'refPath-val :lambda-list '(m))
(cl:defmethod refPath-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:refPath-val is deprecated.  Use cav_msgs-msg:refPath instead.")
  (refPath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goal>) ostream)
  "Serializes a message object of type '<Goal>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'Estop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'Go) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'refPath))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'refPath))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goal>) istream)
  "Deserializes a message object of type '<Goal>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Estop) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'Go) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'refPath) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'refPath)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cav_msgs-msg:RefPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goal>)))
  "Returns string type for a message object of type '<Goal>"
  "cav_msgs/Goal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goal)))
  "Returns string type for a message object of type 'Goal"
  "cav_msgs/Goal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goal>)))
  "Returns md5sum for a message object of type '<Goal>"
  "5512e5dddfea9dde6ed8bf22e640c5e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goal)))
  "Returns md5sum for a message object of type 'Goal"
  "5512e5dddfea9dde6ed8bf22e640c5e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goal>)))
  "Returns full string definition for message of type '<Goal>"
  (cl:format cl:nil "~%##timestamp, ros time or utc time~%float64     timestamp~%~%#part 1: direct commands~%int16 	Estop    # 0:none  1:low  2:medium   3:high~%bool 	Go       # false, true~%~%#part 2 -- lane center and drivable area~%RefPoint[] refPath~%~%~%================================================================================~%MSG: cav_msgs/RefPoint~%#reference path~%float64 t   # time of this point~%float64 x~%float64 y~%float64 v~%float64 heading~%float64 cr~%~%#drivable area~%float64 	left_width~%float64 	right_width~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goal)))
  "Returns full string definition for message of type 'Goal"
  (cl:format cl:nil "~%##timestamp, ros time or utc time~%float64     timestamp~%~%#part 1: direct commands~%int16 	Estop    # 0:none  1:low  2:medium   3:high~%bool 	Go       # false, true~%~%#part 2 -- lane center and drivable area~%RefPoint[] refPath~%~%~%================================================================================~%MSG: cav_msgs/RefPoint~%#reference path~%float64 t   # time of this point~%float64 x~%float64 y~%float64 v~%float64 heading~%float64 cr~%~%#drivable area~%float64 	left_width~%float64 	right_width~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goal>))
  (cl:+ 0
     8
     2
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'refPath) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goal>))
  "Converts a ROS message object to a list"
  (cl:list 'Goal
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':Estop (Estop msg))
    (cl:cons ':Go (Go msg))
    (cl:cons ':refPath (refPath msg))
))
