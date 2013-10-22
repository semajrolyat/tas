; Auto-generated. Do not edit!


(cl:in-package pendulum-srv)


;//! \htmlinclude command-request.msg.html

(cl:defclass <command-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass command-request (<command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-srv:<command-request> is deprecated: use pendulum-srv:command-request instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:time-val is deprecated.  Use pendulum-srv:time instead.")
  (time m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:position-val is deprecated.  Use pendulum-srv:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:velocity-val is deprecated.  Use pendulum-srv:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <command-request>) ostream)
  "Serializes a message object of type '<command-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <command-request>) istream)
  "Deserializes a message object of type '<command-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<command-request>)))
  "Returns string type for a service object of type '<command-request>"
  "pendulum/commandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'command-request)))
  "Returns string type for a service object of type 'command-request"
  "pendulum/commandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<command-request>)))
  "Returns md5sum for a message object of type '<command-request>"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'command-request)))
  "Returns md5sum for a message object of type 'command-request"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<command-request>)))
  "Returns full string definition for message of type '<command-request>"
  (cl:format cl:nil "float64 time~%float64 position~%float64 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'command-request)))
  "Returns full string definition for message of type 'command-request"
  (cl:format cl:nil "float64 time~%float64 position~%float64 velocity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <command-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'command-request
    (cl:cons ':time (time msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
))
;//! \htmlinclude command-response.msg.html

(cl:defclass <command-response> (roslisp-msg-protocol:ros-message)
  ((torque
    :reader torque
    :initarg :torque
    :type cl:float
    :initform 0.0))
)

(cl:defclass command-response (<command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-srv:<command-response> is deprecated: use pendulum-srv:command-response instead.")))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-srv:torque-val is deprecated.  Use pendulum-srv:torque instead.")
  (torque m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <command-response>) ostream)
  "Serializes a message object of type '<command-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <command-response>) istream)
  "Deserializes a message object of type '<command-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<command-response>)))
  "Returns string type for a service object of type '<command-response>"
  "pendulum/commandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'command-response)))
  "Returns string type for a service object of type 'command-response"
  "pendulum/commandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<command-response>)))
  "Returns md5sum for a message object of type '<command-response>"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'command-response)))
  "Returns md5sum for a message object of type 'command-response"
  "7476d6dcbd80f4b9d1318d94e878a310")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<command-response>)))
  "Returns full string definition for message of type '<command-response>"
  (cl:format cl:nil "float64 torque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'command-response)))
  "Returns full string definition for message of type 'command-response"
  (cl:format cl:nil "float64 torque~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <command-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'command-response
    (cl:cons ':torque (torque msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'command)))
  'command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'command)))
  'command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'command)))
  "Returns string type for a service object of type '<command>"
  "pendulum/command")