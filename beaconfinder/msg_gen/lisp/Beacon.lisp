; Auto-generated. Do not edit!


(cl:in-package beaconfinder-msg)


;//! \htmlinclude Beacon.msg.html

(cl:defclass <Beacon> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass Beacon (<Beacon>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Beacon>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Beacon)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beaconfinder-msg:<Beacon> is deprecated: use beaconfinder-msg:Beacon instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <Beacon>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beaconfinder-msg:ID-val is deprecated.  Use beaconfinder-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Beacon>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beaconfinder-msg:x-val is deprecated.  Use beaconfinder-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Beacon>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beaconfinder-msg:y-val is deprecated.  Use beaconfinder-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Beacon>) ostream)
  "Serializes a message object of type '<Beacon>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Beacon>) istream)
  "Deserializes a message object of type '<Beacon>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ID)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Beacon>)))
  "Returns string type for a message object of type '<Beacon>"
  "beaconfinder/Beacon")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Beacon)))
  "Returns string type for a message object of type 'Beacon"
  "beaconfinder/Beacon")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Beacon>)))
  "Returns md5sum for a message object of type '<Beacon>"
  "84629ebbc444e13ee22538b56befaad9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Beacon)))
  "Returns md5sum for a message object of type 'Beacon"
  "84629ebbc444e13ee22538b56befaad9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Beacon>)))
  "Returns full string definition for message of type '<Beacon>"
  (cl:format cl:nil "# Beacon message type, for a single beacon~%~%# These IDs go from 0..n, for n beacons in increasing order of size~%uint8 ID~%~%# The x coordinate (robot relative) of the centre of the beacon~%float32 x~%~%# The y coordinate (robot relative) of the centre of the beacon~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Beacon)))
  "Returns full string definition for message of type 'Beacon"
  (cl:format cl:nil "# Beacon message type, for a single beacon~%~%# These IDs go from 0..n, for n beacons in increasing order of size~%uint8 ID~%~%# The x coordinate (robot relative) of the centre of the beacon~%float32 x~%~%# The y coordinate (robot relative) of the centre of the beacon~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Beacon>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Beacon>))
  "Converts a ROS message object to a list"
  (cl:list 'Beacon
    (cl:cons ':ID (ID msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
