// Auto-generated. Do not edit!

// (in-package cav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point = require('./Point.js');

//-----------------------------------------------------------

class PlanedPath {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.Go = null;
      this.Estop = null;
      this.forward_flag = null;
      this.backward_flag = null;
      this.planedPath = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('Go')) {
        this.Go = initObj.Go
      }
      else {
        this.Go = false;
      }
      if (initObj.hasOwnProperty('Estop')) {
        this.Estop = initObj.Estop
      }
      else {
        this.Estop = 0;
      }
      if (initObj.hasOwnProperty('forward_flag')) {
        this.forward_flag = initObj.forward_flag
      }
      else {
        this.forward_flag = false;
      }
      if (initObj.hasOwnProperty('backward_flag')) {
        this.backward_flag = initObj.backward_flag
      }
      else {
        this.backward_flag = false;
      }
      if (initObj.hasOwnProperty('planedPath')) {
        this.planedPath = initObj.planedPath
      }
      else {
        this.planedPath = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanedPath
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [Go]
    bufferOffset = _serializer.bool(obj.Go, buffer, bufferOffset);
    // Serialize message field [Estop]
    bufferOffset = _serializer.int16(obj.Estop, buffer, bufferOffset);
    // Serialize message field [forward_flag]
    bufferOffset = _serializer.bool(obj.forward_flag, buffer, bufferOffset);
    // Serialize message field [backward_flag]
    bufferOffset = _serializer.bool(obj.backward_flag, buffer, bufferOffset);
    // Serialize message field [planedPath]
    // Serialize the length for message field [planedPath]
    bufferOffset = _serializer.uint32(obj.planedPath.length, buffer, bufferOffset);
    obj.planedPath.forEach((val) => {
      bufferOffset = Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanedPath
    let len;
    let data = new PlanedPath(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Go]
    data.Go = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [Estop]
    data.Estop = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [forward_flag]
    data.forward_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [backward_flag]
    data.backward_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [planedPath]
    // Deserialize array length for message field [planedPath]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.planedPath = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.planedPath[i] = Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 48 * object.planedPath.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_msgs/PlanedPath';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c81ebe469d3c8540c20f292a56d467d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanedPath(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.Go !== undefined) {
      resolved.Go = msg.Go;
    }
    else {
      resolved.Go = false
    }

    if (msg.Estop !== undefined) {
      resolved.Estop = msg.Estop;
    }
    else {
      resolved.Estop = 0
    }

    if (msg.forward_flag !== undefined) {
      resolved.forward_flag = msg.forward_flag;
    }
    else {
      resolved.forward_flag = false
    }

    if (msg.backward_flag !== undefined) {
      resolved.backward_flag = msg.backward_flag;
    }
    else {
      resolved.backward_flag = false
    }

    if (msg.planedPath !== undefined) {
      resolved.planedPath = new Array(msg.planedPath.length);
      for (let i = 0; i < resolved.planedPath.length; ++i) {
        resolved.planedPath[i] = Point.Resolve(msg.planedPath[i]);
      }
    }
    else {
      resolved.planedPath = []
    }

    return resolved;
    }
};

module.exports = PlanedPath;
