; Auto-generated. Do not edit!


(cl:in-package movement-msg)


;//! \htmlinclude wheelVelocity.msg.html

(cl:defclass <wheelVelocity> (roslisp-msg-protocol:ros-message)
  ((lwheel
    :reader lwheel
    :initarg :lwheel
    :type cl:float
    :initform 0.0)
   (rwheel
    :reader rwheel
    :initarg :rwheel
    :type cl:float
    :initform 0.0))
)

(cl:defclass wheelVelocity (<wheelVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wheelVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wheelVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name movement-msg:<wheelVelocity> is deprecated: use movement-msg:wheelVelocity instead.")))

(cl:ensure-generic-function 'lwheel-val :lambda-list '(m))
(cl:defmethod lwheel-val ((m <wheelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movement-msg:lwheel-val is deprecated.  Use movement-msg:lwheel instead.")
  (lwheel m))

(cl:ensure-generic-function 'rwheel-val :lambda-list '(m))
(cl:defmethod rwheel-val ((m <wheelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader movement-msg:rwheel-val is deprecated.  Use movement-msg:rwheel instead.")
  (rwheel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wheelVelocity>) ostream)
  "Serializes a message object of type '<wheelVelocity>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'lwheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rwheel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wheelVelocity>) istream)
  "Deserializes a message object of type '<wheelVelocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lwheel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rwheel) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wheelVelocity>)))
  "Returns string type for a message object of type '<wheelVelocity>"
  "movement/wheelVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wheelVelocity)))
  "Returns string type for a message object of type 'wheelVelocity"
  "movement/wheelVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wheelVelocity>)))
  "Returns md5sum for a message object of type '<wheelVelocity>"
  "e22920fde66adfb9b293a1db4c491138")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wheelVelocity)))
  "Returns md5sum for a message object of type 'wheelVelocity"
  "e22920fde66adfb9b293a1db4c491138")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wheelVelocity>)))
  "Returns full string definition for message of type '<wheelVelocity>"
  (cl:format cl:nil "float32 lwheel~%float32 rwheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wheelVelocity)))
  "Returns full string definition for message of type 'wheelVelocity"
  (cl:format cl:nil "float32 lwheel~%float32 rwheel~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wheelVelocity>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wheelVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'wheelVelocity
    (cl:cons ':lwheel (lwheel msg))
    (cl:cons ':rwheel (rwheel msg))
))
