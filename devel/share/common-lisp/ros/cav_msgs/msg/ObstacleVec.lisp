; Auto-generated. Do not edit!


(cl:in-package cav_msgs-msg)


;//! \htmlinclude ObstacleVec.msg.html

(cl:defclass <ObstacleVec> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0)
   (len
    :reader len
    :initarg :len
    :type cl:integer
    :initform 0)
   (obstacleVec
    :reader obstacleVec
    :initarg :obstacleVec
    :type (cl:vector cav_msgs-msg:Obstacle)
   :initform (cl:make-array 0 :element-type 'cav_msgs-msg:Obstacle :initial-element (cl:make-instance 'cav_msgs-msg:Obstacle))))
)

(cl:defclass ObstacleVec (<ObstacleVec>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleVec>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleVec)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cav_msgs-msg:<ObstacleVec> is deprecated: use cav_msgs-msg:ObstacleVec instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <ObstacleVec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:timestamp-val is deprecated.  Use cav_msgs-msg:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'len-val :lambda-list '(m))
(cl:defmethod len-val ((m <ObstacleVec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:len-val is deprecated.  Use cav_msgs-msg:len instead.")
  (len m))

(cl:ensure-generic-function 'obstacleVec-val :lambda-list '(m))
(cl:defmethod obstacleVec-val ((m <ObstacleVec>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cav_msgs-msg:obstacleVec-val is deprecated.  Use cav_msgs-msg:obstacleVec instead.")
  (obstacleVec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleVec>) ostream)
  "Serializes a message object of type '<ObstacleVec>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'len)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacleVec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacleVec))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleVec>) istream)
  "Deserializes a message object of type '<ObstacleVec>"
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
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'len) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacleVec) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacleVec)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cav_msgs-msg:Obstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleVec>)))
  "Returns string type for a message object of type '<ObstacleVec>"
  "cav_msgs/ObstacleVec")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleVec)))
  "Returns string type for a message object of type 'ObstacleVec"
  "cav_msgs/ObstacleVec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleVec>)))
  "Returns md5sum for a message object of type '<ObstacleVec>"
  "60d3a37403fa2a27da845f05fbc1cd50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleVec)))
  "Returns md5sum for a message object of type 'ObstacleVec"
  "60d3a37403fa2a27da845f05fbc1cd50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleVec>)))
  "Returns full string definition for message of type '<ObstacleVec>"
  (cl:format cl:nil "~%float64 	timestamp~%int32   	len~%Obstacle[]   	obstacleVec~%~%================================================================================~%MSG: cav_msgs/Obstacle~%float64 	timestamp   #ros time~%int32 		id~%int16 		type        #refer to below definition       ~%float64     radius      # m, radius for object circle~%~%float64 	x_global   #m, in the same coordinate system of the ego car~%float64 	y_global   #m, in the same coordinate system of the ego car~%~%float64 	speed_x    #not official definition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleVec)))
  "Returns full string definition for message of type 'ObstacleVec"
  (cl:format cl:nil "~%float64 	timestamp~%int32   	len~%Obstacle[]   	obstacleVec~%~%================================================================================~%MSG: cav_msgs/Obstacle~%float64 	timestamp   #ros time~%int32 		id~%int16 		type        #refer to below definition       ~%float64     radius      # m, radius for object circle~%~%float64 	x_global   #m, in the same coordinate system of the ego car~%float64 	y_global   #m, in the same coordinate system of the ego car~%~%float64 	speed_x    #not official definition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleVec>))
  (cl:+ 0
     8
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacleVec) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleVec>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleVec
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':len (len msg))
    (cl:cons ':obstacleVec (obstacleVec msg))
))
