; Auto-generated. Do not edit!


(cl:in-package drone_test-msg)


;//! \htmlinclude code.msg.html

(cl:defclass <code> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type cl:fixnum
    :initform 0))
)

(cl:defclass code (<code>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <code>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'code)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_test-msg:<code> is deprecated: use drone_test-msg:code instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <code>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:code-val is deprecated.  Use drone_test-msg:code instead.")
  (code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <code>) ostream)
  "Serializes a message object of type '<code>"
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <code>) istream)
  "Deserializes a message object of type '<code>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<code>)))
  "Returns string type for a message object of type '<code>"
  "drone_test/code")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'code)))
  "Returns string type for a message object of type 'code"
  "drone_test/code")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<code>)))
  "Returns md5sum for a message object of type '<code>"
  "95cfa23476470f3d4705c11337b96909")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'code)))
  "Returns md5sum for a message object of type 'code"
  "95cfa23476470f3d4705c11337b96909")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<code>)))
  "Returns full string definition for message of type '<code>"
  (cl:format cl:nil "int8 code~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'code)))
  "Returns full string definition for message of type 'code"
  (cl:format cl:nil "int8 code~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <code>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <code>))
  "Converts a ROS message object to a list"
  (cl:list 'code
    (cl:cons ':code (code msg))
))
