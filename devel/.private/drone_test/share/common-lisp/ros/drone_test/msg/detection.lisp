; Auto-generated. Do not edit!


(cl:in-package drone_test-msg)


;//! \htmlinclude detection.msg.html

(cl:defclass <detection> (roslisp-msg-protocol:ros-message)
  ((erro_x
    :reader erro_x
    :initarg :erro_x
    :type cl:float
    :initform 0.0)
   (erro_y
    :reader erro_y
    :initarg :erro_y
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:fixnum
    :initform 0)
   (start_opencv
    :reader start_opencv
    :initarg :start_opencv
    :type cl:fixnum
    :initform 0))
)

(cl:defclass detection (<detection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_test-msg:<detection> is deprecated: use drone_test-msg:detection instead.")))

(cl:ensure-generic-function 'erro_x-val :lambda-list '(m))
(cl:defmethod erro_x-val ((m <detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:erro_x-val is deprecated.  Use drone_test-msg:erro_x instead.")
  (erro_x m))

(cl:ensure-generic-function 'erro_y-val :lambda-list '(m))
(cl:defmethod erro_y-val ((m <detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:erro_y-val is deprecated.  Use drone_test-msg:erro_y instead.")
  (erro_y m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:flag-val is deprecated.  Use drone_test-msg:flag instead.")
  (flag m))

(cl:ensure-generic-function 'start_opencv-val :lambda-list '(m))
(cl:defmethod start_opencv-val ((m <detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:start_opencv-val is deprecated.  Use drone_test-msg:start_opencv instead.")
  (start_opencv m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detection>) ostream)
  "Serializes a message object of type '<detection>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'erro_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'erro_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'flag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'start_opencv)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detection>) istream)
  "Deserializes a message object of type '<detection>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'erro_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'erro_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flag) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_opencv) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detection>)))
  "Returns string type for a message object of type '<detection>"
  "drone_test/detection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detection)))
  "Returns string type for a message object of type 'detection"
  "drone_test/detection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detection>)))
  "Returns md5sum for a message object of type '<detection>"
  "5758eab71db35db21ff4cedc29365b18")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detection)))
  "Returns md5sum for a message object of type 'detection"
  "5758eab71db35db21ff4cedc29365b18")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detection>)))
  "Returns full string definition for message of type '<detection>"
  (cl:format cl:nil "float64 erro_x~%float64 erro_y~%int8 flag~%int8 start_opencv~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detection)))
  "Returns full string definition for message of type 'detection"
  (cl:format cl:nil "float64 erro_x~%float64 erro_y~%int8 flag~%int8 start_opencv~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detection>))
  (cl:+ 0
     8
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detection>))
  "Converts a ROS message object to a list"
  (cl:list 'detection
    (cl:cons ':erro_x (erro_x msg))
    (cl:cons ':erro_y (erro_y msg))
    (cl:cons ':flag (flag msg))
    (cl:cons ':start_opencv (start_opencv msg))
))
