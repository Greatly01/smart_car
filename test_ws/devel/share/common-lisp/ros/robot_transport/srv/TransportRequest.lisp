; Auto-generated. Do not edit!


(cl:in-package robot_transport-srv)


;//! \htmlinclude TransportRequest-request.msg.html

(cl:defclass <TransportRequest-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type cl:string
    :initform ""))
)

(cl:defclass TransportRequest-request (<TransportRequest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransportRequest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransportRequest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_transport-srv:<TransportRequest-request> is deprecated: use robot_transport-srv:TransportRequest-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <TransportRequest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_transport-srv:target-val is deprecated.  Use robot_transport-srv:target instead.")
  (target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransportRequest-request>) ostream)
  "Serializes a message object of type '<TransportRequest-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'target))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'target))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransportRequest-request>) istream)
  "Deserializes a message object of type '<TransportRequest-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'target) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransportRequest-request>)))
  "Returns string type for a service object of type '<TransportRequest-request>"
  "robot_transport/TransportRequestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransportRequest-request)))
  "Returns string type for a service object of type 'TransportRequest-request"
  "robot_transport/TransportRequestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransportRequest-request>)))
  "Returns md5sum for a message object of type '<TransportRequest-request>"
  "e41a7a3cc3ea192648ce06af33a5d757")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransportRequest-request)))
  "Returns md5sum for a message object of type 'TransportRequest-request"
  "e41a7a3cc3ea192648ce06af33a5d757")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransportRequest-request>)))
  "Returns full string definition for message of type '<TransportRequest-request>"
  (cl:format cl:nil "string target  # 请求参数：目标标记（A/B/C/D/CAMP/WAIT）~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransportRequest-request)))
  "Returns full string definition for message of type 'TransportRequest-request"
  (cl:format cl:nil "string target  # 请求参数：目标标记（A/B/C/D/CAMP/WAIT）~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransportRequest-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransportRequest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TransportRequest-request
    (cl:cons ':target (target msg))
))
;//! \htmlinclude TransportRequest-response.msg.html

(cl:defclass <TransportRequest-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass TransportRequest-response (<TransportRequest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TransportRequest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TransportRequest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_transport-srv:<TransportRequest-response> is deprecated: use robot_transport-srv:TransportRequest-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TransportRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_transport-srv:success-val is deprecated.  Use robot_transport-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TransportRequest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_transport-srv:message-val is deprecated.  Use robot_transport-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TransportRequest-response>) ostream)
  "Serializes a message object of type '<TransportRequest-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TransportRequest-response>) istream)
  "Deserializes a message object of type '<TransportRequest-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TransportRequest-response>)))
  "Returns string type for a service object of type '<TransportRequest-response>"
  "robot_transport/TransportRequestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransportRequest-response)))
  "Returns string type for a service object of type 'TransportRequest-response"
  "robot_transport/TransportRequestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TransportRequest-response>)))
  "Returns md5sum for a message object of type '<TransportRequest-response>"
  "e41a7a3cc3ea192648ce06af33a5d757")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TransportRequest-response)))
  "Returns md5sum for a message object of type 'TransportRequest-response"
  "e41a7a3cc3ea192648ce06af33a5d757")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TransportRequest-response>)))
  "Returns full string definition for message of type '<TransportRequest-response>"
  (cl:format cl:nil "bool success   # 响应结果：是否成功~%string message # 响应消息：状态描述~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TransportRequest-response)))
  "Returns full string definition for message of type 'TransportRequest-response"
  (cl:format cl:nil "bool success   # 响应结果：是否成功~%string message # 响应消息：状态描述~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TransportRequest-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TransportRequest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TransportRequest-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TransportRequest)))
  'TransportRequest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TransportRequest)))
  'TransportRequest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TransportRequest)))
  "Returns string type for a service object of type '<TransportRequest>"
  "robot_transport/TransportRequest")