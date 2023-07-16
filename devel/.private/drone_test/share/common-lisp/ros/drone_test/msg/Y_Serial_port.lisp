; Auto-generated. Do not edit!


(cl:in-package drone_test-msg)


;//! \htmlinclude Y_Serial_port.msg.html

(cl:defclass <Y_Serial_port> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (tar1
    :reader tar1
    :initarg :tar1
    :type cl:fixnum
    :initform 0)
   (tar2
    :reader tar2
    :initarg :tar2
    :type cl:fixnum
    :initform 0)
   (if_takeoff
    :reader if_takeoff
    :initarg :if_takeoff
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Y_Serial_port (<Y_Serial_port>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Y_Serial_port>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Y_Serial_port)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_test-msg:<Y_Serial_port> is deprecated: use drone_test-msg:Y_Serial_port instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Y_Serial_port>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:mode-val is deprecated.  Use drone_test-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'tar1-val :lambda-list '(m))
(cl:defmethod tar1-val ((m <Y_Serial_port>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:tar1-val is deprecated.  Use drone_test-msg:tar1 instead.")
  (tar1 m))

(cl:ensure-generic-function 'tar2-val :lambda-list '(m))
(cl:defmethod tar2-val ((m <Y_Serial_port>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:tar2-val is deprecated.  Use drone_test-msg:tar2 instead.")
  (tar2 m))

(cl:ensure-generic-function 'if_takeoff-val :lambda-list '(m))
(cl:defmethod if_takeoff-val ((m <Y_Serial_port>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_test-msg:if_takeoff-val is deprecated.  Use drone_test-msg:if_takeoff instead.")
  (if_takeoff m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Y_Serial_port>) ostream)
  "Serializes a message object of type '<Y_Serial_port>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tar1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'tar2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'if_takeoff)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Y_Serial_port>) istream)
  "Deserializes a message object of type '<Y_Serial_port>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tar1) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tar2) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'if_takeoff) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Y_Serial_port>)))
  "Returns string type for a message object of type '<Y_Serial_port>"
  "drone_test/Y_Serial_port")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Y_Serial_port)))
  "Returns string type for a message object of type 'Y_Serial_port"
  "drone_test/Y_Serial_port")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Y_Serial_port>)))
  "Returns md5sum for a message object of type '<Y_Serial_port>"
  "ae4cda509b27984341a3ec103b7367d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Y_Serial_port)))
  "Returns md5sum for a message object of type 'Y_Serial_port"
  "ae4cda509b27984341a3ec103b7367d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Y_Serial_port>)))
  "Returns full string definition for message of type '<Y_Serial_port>"
  (cl:format cl:nil "int8 mode~%int8 tar1~%int8 tar2~%int8 if_takeoff ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Y_Serial_port)))
  "Returns full string definition for message of type 'Y_Serial_port"
  (cl:format cl:nil "int8 mode~%int8 tar1~%int8 tar2~%int8 if_takeoff ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Y_Serial_port>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Y_Serial_port>))
  "Converts a ROS message object to a list"
  (cl:list 'Y_Serial_port
    (cl:cons ':mode (mode msg))
    (cl:cons ':tar1 (tar1 msg))
    (cl:cons ':tar2 (tar2 msg))
    (cl:cons ':if_takeoff (if_takeoff msg))
))
