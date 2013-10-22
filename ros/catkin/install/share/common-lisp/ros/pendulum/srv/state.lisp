; Auto-generated. Do not edit!


(cl:in-package pendulum-srv)


;//! \htmlinclude state-request.msg.html

(cl:defclass <state-request> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:float
    :initform 0.0))
)

(cl:defclass state-request (<state-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-srv:<state-request> is deprecated: use pendulum-srv:state-request instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <state-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:timestamp-val is deprecated.  Use pendulum-srv:timestamp instead.")
  (timestamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state-request>) ostream)
  "Serializes a message object of type '<state-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timestamp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state-request>) istream)
  "Deserializes a message object of type '<state-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state-request>)))
  "Returns string type for a service object of type '<state-request>"
  "pendulum/stateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state-request)))
  "Returns string type for a service object of type 'state-request"
  "pendulum/stateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state-request>)))
  "Returns md5sum for a message object of type '<state-request>"
  "5df9c4cc22276ed3d7c49b8b3485993e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state-request)))
  "Returns md5sum for a message object of type 'state-request"
  "5df9c4cc22276ed3d7c49b8b3485993e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state-request>)))
  "Returns full string definition for message of type '<state-request>"
  (cl:format cl:nil "float64 timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state-request)))
  "Returns full string definition for message of type 'state-request"
  (cl:format cl:nil "float64 timestamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state-request>))
  "Converts a ROS message object to a list"
  (cl:list 'state-request
    (cl:cons ':timestamp (timestamp msg))
))
;//! \htmlinclude state-response.msg.html

(cl:defclass <state-response> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (position
    :reader position
    :initarg :position
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0))
)

(cl:defclass state-response (<state-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-srv:<state-response> is deprecated: use pendulum-srv:state-response instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:time-val is deprecated.  Use pendulum-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:position-val is deprecated.  Use pendulum-srv:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <state-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:velocity-val is deprecated.  Use pendulum-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state-response>) ostream)
  "Serializes a message object of type '<state-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state-response>) istream)
  "Deserializes a message object of type '<state-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state-response>)))
  "Returns string type for a service object of type '<state-response>"
  "pendulum/stateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state-response)))
  "Returns string type for a service object of type 'state-response"
  "pendulum/stateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state-response>)))
  "Returns md5sum for a message object of type '<state-response>"
  "5df9c4cc22276ed3d7c49b8b3485993e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state-response)))
  "Returns md5sum for a message object of type 'state-response"
  "5df9c4cc22276ed3d7c49b8b3485993e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state-response>)))
  "Returns full string definition for message of type '<state-response>"
  (cl:format cl:nil "float64 time~%float64 position~%float64 velocity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state-response)))
  "Returns full string definition for message of type 'state-response"
  (cl:format cl:nil "float64 time~%float64 position~%float64 velocity~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state-response>))
  "Converts a ROS message object to a list"
  (cl:list 'state-response
    (cl:cons ':time (time msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'state)))
  'state-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'state)))
  'state-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state)))
  "Returns string type for a service object of type '<state>"
  "pendulum/state")