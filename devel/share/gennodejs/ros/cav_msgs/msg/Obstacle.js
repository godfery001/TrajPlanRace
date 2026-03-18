// Auto-generated. Do not edit!

// (in-package cav_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Obstacle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.timestamp = null;
      this.id = null;
      this.type = null;
      this.radius = null;
      this.x_global = null;
      this.y_global = null;
      this.speed_x = null;
    }
    else {
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('x_global')) {
        this.x_global = initObj.x_global
      }
      else {
        this.x_global = 0.0;
      }
      if (initObj.hasOwnProperty('y_global')) {
        this.y_global = initObj.y_global
      }
      else {
        this.y_global = 0.0;
      }
      if (initObj.hasOwnProperty('speed_x')) {
        this.speed_x = initObj.speed_x
      }
      else {
        this.speed_x = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Obstacle
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int16(obj.type, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    // Serialize message field [x_global]
    bufferOffset = _serializer.float64(obj.x_global, buffer, bufferOffset);
    // Serialize message field [y_global]
    bufferOffset = _serializer.float64(obj.y_global, buffer, bufferOffset);
    // Serialize message field [speed_x]
    bufferOffset = _serializer.float64(obj.speed_x, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Obstacle
    let len;
    let data = new Obstacle(null);
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [x_global]
    data.x_global = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_global]
    data.y_global = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed_x]
    data.speed_x = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 46;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cav_msgs/Obstacle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a54a7e58941fdabf5915ce1f4cd97ed5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Obstacle(null);
    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.x_global !== undefined) {
      resolved.x_global = msg.x_global;
    }
    else {
      resolved.x_global = 0.0
    }

    if (msg.y_global !== undefined) {
      resolved.y_global = msg.y_global;
    }
    else {
      resolved.y_global = 0.0
    }

    if (msg.speed_x !== undefined) {
      resolved.speed_x = msg.speed_x;
    }
    else {
      resolved.speed_x = 0.0
    }

    return resolved;
    }
};

module.exports = Obstacle;
