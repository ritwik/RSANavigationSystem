; Auto-generated. Do not edit!


(cl:in-package beaconfinder-msg)


;//! \htmlinclude Beacons.msg.html

(cl:defclass <Beacons> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (numBeacons
    :reader numBeacons
    :initarg :numBeacons
    :type cl:fixnum
    :initform 0)
   (beacon
    :reader beacon
    :initarg :beacon
    :type (cl:vector beaconfinder-msg:Beacon)
   :initform (cl:make-array 0 :element-type 'beaconfinder-msg:Beacon :initial-element (cl:make-instance 'beaconfinder-msg:Beacon))))
)

(cl:defclass Beacons (<Beacons>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Beacons>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Beacons)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beaconfinder-msg:<Beacons> is deprecated: use beaconfinder-msg:Beacons instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Beacons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beaconfinder-msg:header-val is deprecated.  Use beaconfinder-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'numBeacons-val :lambda-list '(m))
(cl:defmethod numBeacons-val ((m <Beacons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beaconfinder-msg:numBeacons-val is deprecated.  Use beaconfinder-msg:numBeacons instead.")
  (numBeacons m))

(cl:ensure-generic-function 'beacon-val :lambda-list '(m))
(cl:defmethod beacon-val ((m <Beacons>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beaconfinder-msg:beacon-val is deprecated.  Use beaconfinder-msg:beacon instead.")
  (beacon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Beacons>) ostream)
  "Serializes a message object of type '<Beacons>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numBeacons)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'beacon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'beacon))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Beacons>) istream)
  "Deserializes a message object of type '<Beacons>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numBeacons)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'beacon) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'beacon)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'beaconfinder-msg:Beacon))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Beacons>)))
  "Returns string type for a message object of type '<Beacons>"
  "beaconfinder/Beacons")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Beacons)))
  "Returns string type for a message object of type 'Beacons"
  "beaconfinder/Beacons")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Beacons>)))
  "Returns md5sum for a message object of type '<Beacons>"
  "873d03fb58b312326f8bc5e4bda6e56b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Beacons)))
  "Returns md5sum for a message object of type 'Beacons"
  "873d03fb58b312326f8bc5e4bda6e56b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Beacons>)))
  "Returns full string definition for message of type '<Beacons>"
  (cl:format cl:nil "#Beacons message for publishing a list of beacons from a laser scan to a topic~%Header header~%uint8 numBeacons~%# A list of Beacons found at this timestamp~%Beacon[] beacon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: beaconfinder/Beacon~%# Beacon message type, for a single beacon~%~%# These IDs go from 0..n, for n beacons in increasing order of size~%uint8 ID~%~%# The x coordinate (robot relative) of the centre of the beacon~%float32 x~%~%# The y coordinate (robot relative) of the centre of the beacon~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Beacons)))
  "Returns full string definition for message of type 'Beacons"
  (cl:format cl:nil "#Beacons message for publishing a list of beacons from a laser scan to a topic~%Header header~%uint8 numBeacons~%# A list of Beacons found at this timestamp~%Beacon[] beacon~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: beaconfinder/Beacon~%# Beacon message type, for a single beacon~%~%# These IDs go from 0..n, for n beacons in increasing order of size~%uint8 ID~%~%# The x coordinate (robot relative) of the centre of the beacon~%float32 x~%~%# The y coordinate (robot relative) of the centre of the beacon~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Beacons>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'beacon) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Beacons>))
  "Converts a ROS message object to a list"
  (cl:list 'Beacons
    (cl:cons ':header (header msg))
    (cl:cons ':numBeacons (numBeacons msg))
    (cl:cons ':beacon (beacon msg))
))
