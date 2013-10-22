; Auto-generated. Do not edit!


(cl:in-package pendulum-msg)


;//! \htmlinclude synch.msg.html

(cl:defclass <synch> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (msg
    :reader msg
    :initarg :msg
    :type cl:integer
    :initform 0))
)

(cl:defclass synch (<synch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <synch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'synch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pendulum-msg:<synch> is deprecated: use pendulum-msg:synch instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <synch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-msg:time-val is deprecated.  Use pendulum-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <synch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pendulum-msg:msg-val is deprecated.  Use pendulum-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <synch>) ostream)
  "Serializes a message object of type '<synch>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <synch>) istream)
  "Deserializes a message object of type '<synch>"
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'msg)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'msg)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<synch>)))
  "Returns string type for a message object of type '<synch>"
  "pendulum/synch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'synch)))
  "Returns string type for a message object of type 'synch"
  "pendulum/synch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<synch>)))
  "Returns md5sum for a message object of type '<synch>"
  "71c73720e1650008d117b8fc56ad1f04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'synch)))
  "Returns md5sum for a message object of type 'synch"
  "71c73720e1650008d117b8fc56ad1f04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<synch>)))
  "Returns full string definition for message of type '<synch>"
  (cl:format cl:nil "float64 time~%uint32 msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'synch)))
  "Returns full string definition for message of type 'synch"
  (cl:format cl:nil "float64 time~%uint32 msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <synch>))
  (cl:+ 0
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <synch>))
  "Converts a ROS message object to a list"
  (cl:list 'synch
    (cl:cons ':time (time msg))
    (cl:cons ':msg (msg msg))
))
