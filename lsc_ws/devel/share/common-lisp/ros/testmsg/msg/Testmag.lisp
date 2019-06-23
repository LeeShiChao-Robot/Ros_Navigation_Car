; Auto-generated. Do not edit!


(cl:in-package testmsg-msg)


;//! \htmlinclude Testmag.msg.html

(cl:defclass <Testmag> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass Testmag (<Testmag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Testmag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Testmag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name testmsg-msg:<Testmag> is deprecated: use testmsg-msg:Testmag instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <Testmag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader testmsg-msg:left-val is deprecated.  Use testmsg-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <Testmag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader testmsg-msg:right-val is deprecated.  Use testmsg-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Testmag>) ostream)
  "Serializes a message object of type '<Testmag>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Testmag>) istream)
  "Deserializes a message object of type '<Testmag>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Testmag>)))
  "Returns string type for a message object of type '<Testmag>"
  "testmsg/Testmag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Testmag)))
  "Returns string type for a message object of type 'Testmag"
  "testmsg/Testmag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Testmag>)))
  "Returns md5sum for a message object of type '<Testmag>"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Testmag)))
  "Returns md5sum for a message object of type 'Testmag"
  "3a927990ab5d5c3d628e2d52b8533e52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Testmag>)))
  "Returns full string definition for message of type '<Testmag>"
  (cl:format cl:nil "float32 left~%float32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Testmag)))
  "Returns full string definition for message of type 'Testmag"
  (cl:format cl:nil "float32 left~%float32 right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Testmag>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Testmag>))
  "Converts a ROS message object to a list"
  (cl:list 'Testmag
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
