// Auto-generated. Do not edit!

// (in-package cav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RefPoint = require('./RefPoint.js');

//-----------------------------------------------------------

class Goal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.Estop = null;
      this.Go = null;
      this.refPath = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('Estop')) {
        this.Estop = initObj.Estop
      }
      else {
        this.Estop = 0;
      }
      if (initObj.hasOwnProperty('Go')) {
        this.Go = initObj.Go
      }
      else {
        this.Go = false;
      }
      if (initObj.hasOwnProperty('refPath')) {
        this.refPath = initObj.refPath
      }
      else {
        this.refPath = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Goal
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [Estop]
    bufferOffset = _serializer.int16(obj.Estop, buffer, bufferOffset);
    // Serialize message field [Go]
    bufferOffset = _serializer.bool(obj.Go, buffer, bufferOffset);
    // Serialize message field [refPath]
    // Serialize the length for message field [refPath]
    bufferOffset = _serializer.uint32(obj.refPath.length, buffer, bufferOffset);
    obj.refPath.forEach((val) => {
      bufferOffset = RefPoint.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Goal
    let len;
    let data = new Goal(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Estop]
    data.Estop = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [Go]
    data.Go = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [refPath]
    // Deserialize array length for message field [refPath]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.refPath = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.refPath[i] = RefPoint.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 64 * object.refPath.length;
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_msgs/Goal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5512e5dddfea9dde6ed8bf22e640c5e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Goal(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.Estop !== undefined) {
      resolved.Estop = msg.Estop;
    }
    else {
      resolved.Estop = 0
    }

    if (msg.Go !== undefined) {
      resolved.Go = msg.Go;
    }
    else {
      resolved.Go = false
    }

    if (msg.refPath !== undefined) {
      resolved.refPath = new Array(msg.refPath.length);
      for (let i = 0; i < resolved.refPath.length; ++i) {
        resolved.refPath[i] = RefPoint.Resolve(msg.refPath[i]);
      }
    }
    else {
      resolved.refPath = []
    }

    return resolved;
    }
};

module.exports = Goal;
