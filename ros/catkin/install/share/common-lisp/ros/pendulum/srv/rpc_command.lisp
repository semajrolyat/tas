; Auto-generated. Do not edit!


(cl:in-package pendulum-srv)


;//! \htmlinclude rpc_command-request.msg.html

(cl:defclass <rpc_command-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass rpc_command-request (<rpc_command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rpc_command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rpc_command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-srv:<rpc_command-request> is deprecated: use pendulum-srv:rpc_command-request instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <rpc_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:time-val is deprecated.  Use pendulum-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <rpc_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:position-val is deprecated.  Use pendulum-srv:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <rpc_command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:velocity-val is deprecated.  Use pendulum-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rpc_command-request>) ostream)
  "Serializes a message object of type '<rpc_command-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rpc_command-request>) istream)
  "Deserializes a message object of type '<rpc_command-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rpc_command-request>)))
  "Returns string type for a service object of type '<rpc_command-request>"
  "pendulum/rpc_commandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rpc_command-request)))
  "Returns string type for a service object of type 'rpc_command-request"
  "pendulum/rpc_commandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rpc_command-request>)))
  "Returns md5sum for a message object of type '<rpc_command-request>"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rpc_command-request)))
  "Returns md5sum for a message object of type 'rpc_command-request"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rpc_command-request>)))
  "Returns full string definition for message of type '<rpc_command-request>"
  (cl:format cl:nil "float64 time~%float64 position~%float64 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rpc_command-request)))
  "Returns full string definition for message of type 'rpc_command-request"
  (cl:format cl:nil "float64 time~%float64 position~%float64 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rpc_command-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rpc_command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rpc_command-request
    (cl:cons ':time (time msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
))
;//! \htmlinclude rpc_command-response.msg.html

(cl:defclass <rpc_command-response> (roslisp-msg-protocol:ros-message)
  ((torque
    :reader torque
    :initarg :torque
    :type cl:float
    :initform 0.0))
)

(cl:defclass rpc_command-response (<rpc_command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rpc_command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rpc_command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-srv:<rpc_command-response> is deprecated: use pendulum-srv:rpc_command-response instead.")))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <rpc_command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:torque-val is deprecated.  Use pendulum-srv:torque instead.")
  (torque m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rpc_command-response>) ostream)
  "Serializes a message object of type '<rpc_command-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rpc_command-response>) istream)
  "Deserializes a message object of type '<rpc_command-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rpc_command-response>)))
  "Returns string type for a service object of type '<rpc_command-response>"
  "pendulum/rpc_commandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rpc_command-response)))
  "Returns string type for a service object of type 'rpc_command-response"
  "pendulum/rpc_commandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rpc_command-response>)))
  "Returns md5sum for a message object of type '<rpc_command-response>"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rpc_command-response)))
  "Returns md5sum for a message object of type 'rpc_command-response"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rpc_command-response>)))
  "Returns full string definition for message of type '<rpc_command-response>"
  (cl:format cl:nil "float64 torque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rpc_command-response)))
  "Returns full string definition for message of type 'rpc_command-response"
  (cl:format cl:nil "float64 torque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rpc_command-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rpc_command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rpc_command-response
    (cl:cons ':torque (torque msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rpc_command)))
  'rpc_command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rpc_command)))
  'rpc_command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rpc_command)))
  "Returns string type for a service object of type '<rpc_command>"
  "pendulum/rpc_command")