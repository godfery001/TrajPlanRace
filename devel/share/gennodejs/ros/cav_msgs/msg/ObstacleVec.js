// Auto-generated. Do not edit!

// (in-package cav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Obstacle = require('./Obstacle.js');

//-----------------------------------------------------------

class ObstacleVec {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.len = null;
      this.obstacleVec = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('len')) {
        this.len = initObj.len
      }
      else {
        this.len = 0;
      }
      if (initObj.hasOwnProperty('obstacleVec')) {
        this.obstacleVec = initObj.obstacleVec
      }
      else {
        this.obstacleVec = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleVec
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [len]
    bufferOffset = _serializer.int32(obj.len, buffer, bufferOffset);
    // Serialize message field [obstacleVec]
    // Serialize the length for message field [obstacleVec]
    bufferOffset = _serializer.uint32(obj.obstacleVec.length, buffer, bufferOffset);
    obj.obstacleVec.forEach((val) => {
      bufferOffset = Obstacle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleVec
    let len;
    let data = new ObstacleVec(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [len]
    data.len = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [obstacleVec]
    // Deserialize array length for message field [obstacleVec]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.obstacleVec = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.obstacleVec[i] = Obstacle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 46 * object.obstacleVec.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_msgs/ObstacleVec';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60d3a37403fa2a27da845f05fbc1cd50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObstacleVec(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.len !== undefined) {
      resolved.len = msg.len;
    }
    else {
      resolved.len = 0
    }

    if (msg.obstacleVec !== undefined) {
      resolved.obstacleVec = new Array(msg.obstacleVec.length);
      for (let i = 0; i < resolved.obstacleVec.length; ++i) {
        resolved.obstacleVec[i] = Obstacle.Resolve(msg.obstacleVec[i]);
      }
    }
    else {
      resolved.obstacleVec = []
    }

    return resolved;
    }
};

module.exports = ObstacleVec;
