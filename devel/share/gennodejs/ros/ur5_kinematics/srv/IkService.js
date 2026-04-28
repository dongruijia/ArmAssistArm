// Auto-generated. Do not edit!

// (in-package ur5_kinematics.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class IkServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_x = null;
      this.target_y = null;
      this.target_z = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.joint_prev = null;
    }
    else {
      if (initObj.hasOwnProperty('target_x')) {
        this.target_x = initObj.target_x
      }
      else {
        this.target_x = 0.0;
      }
      if (initObj.hasOwnProperty('target_y')) {
        this.target_y = initObj.target_y
      }
      else {
        this.target_y = 0.0;
      }
      if (initObj.hasOwnProperty('target_z')) {
        this.target_z = initObj.target_z
      }
      else {
        this.target_z = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('joint_prev')) {
        this.joint_prev = initObj.joint_prev
      }
      else {
        this.joint_prev = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IkServiceRequest
    // Serialize message field [target_x]
    bufferOffset = _serializer.float64(obj.target_x, buffer, bufferOffset);
    // Serialize message field [target_y]
    bufferOffset = _serializer.float64(obj.target_y, buffer, bufferOffset);
    // Serialize message field [target_z]
    bufferOffset = _serializer.float64(obj.target_z, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [joint_prev]
    bufferOffset = _arraySerializer.float64(obj.joint_prev, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IkServiceRequest
    let len;
    let data = new IkServiceRequest(null);
    // Deserialize message field [target_x]
    data.target_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_y]
    data.target_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_z]
    data.target_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_prev]
    data.joint_prev = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_prev.length;
    return length + 52;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur5_kinematics/IkServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ba25169bdf8186777bc63a2f0222627';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 请求数据 Request
    float64 target_x
    float64 target_y
    float64 target_z
    float64 roll
    float64 pitch
    float64 yaw
    float64[] joint_prev
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IkServiceRequest(null);
    if (msg.target_x !== undefined) {
      resolved.target_x = msg.target_x;
    }
    else {
      resolved.target_x = 0.0
    }

    if (msg.target_y !== undefined) {
      resolved.target_y = msg.target_y;
    }
    else {
      resolved.target_y = 0.0
    }

    if (msg.target_z !== undefined) {
      resolved.target_z = msg.target_z;
    }
    else {
      resolved.target_z = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.joint_prev !== undefined) {
      resolved.joint_prev = msg.joint_prev;
    }
    else {
      resolved.joint_prev = []
    }

    return resolved;
    }
};

class IkServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.error_code = null;
      this.max_angle_error_deg = null;
      this.joint_positions = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('error_code')) {
        this.error_code = initObj.error_code
      }
      else {
        this.error_code = 0;
      }
      if (initObj.hasOwnProperty('max_angle_error_deg')) {
        this.max_angle_error_deg = initObj.max_angle_error_deg
      }
      else {
        this.max_angle_error_deg = 0.0;
      }
      if (initObj.hasOwnProperty('joint_positions')) {
        this.joint_positions = initObj.joint_positions
      }
      else {
        this.joint_positions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IkServiceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [error_code]
    bufferOffset = _serializer.int32(obj.error_code, buffer, bufferOffset);
    // Serialize message field [max_angle_error_deg]
    bufferOffset = _serializer.float64(obj.max_angle_error_deg, buffer, bufferOffset);
    // Serialize message field [joint_positions]
    bufferOffset = _arraySerializer.float64(obj.joint_positions, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IkServiceResponse
    let len;
    let data = new IkServiceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error_code]
    data.error_code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [max_angle_error_deg]
    data.max_angle_error_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [joint_positions]
    data.joint_positions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_positions.length;
    return length + 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur5_kinematics/IkServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a095a0545828f197893a43625f09b4c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 响应数据 Response（必须加 --- 分隔！）
    bool success
    int32 error_code
    float64 max_angle_error_deg
    float64[] joint_positions
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IkServiceResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.error_code !== undefined) {
      resolved.error_code = msg.error_code;
    }
    else {
      resolved.error_code = 0
    }

    if (msg.max_angle_error_deg !== undefined) {
      resolved.max_angle_error_deg = msg.max_angle_error_deg;
    }
    else {
      resolved.max_angle_error_deg = 0.0
    }

    if (msg.joint_positions !== undefined) {
      resolved.joint_positions = msg.joint_positions;
    }
    else {
      resolved.joint_positions = []
    }

    return resolved;
    }
};

module.exports = {
  Request: IkServiceRequest,
  Response: IkServiceResponse,
  md5sum() { return '3ec0d754206886798b3d26a7d01862a6'; },
  datatype() { return 'ur5_kinematics/IkService'; }
};
