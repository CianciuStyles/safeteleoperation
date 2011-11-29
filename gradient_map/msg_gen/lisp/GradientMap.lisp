; Auto-generated. Do not edit!


(cl:in-package gradient_map-msg)


;//! \htmlinclude GradientMap.msg.html

(cl:defclass <GradientMap> (roslisp-msg-protocol:ros-message)
  ((size_x
    :reader size_x
    :initarg :size_x
    :type cl:integer
    :initform 0)
   (size_y
    :reader size_y
    :initarg :size_y
    :type cl:integer
    :initform 0)
   (map
    :reader map
    :initarg :map
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass GradientMap (<GradientMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GradientMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GradientMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gradient_map-msg:<GradientMap> is deprecated: use gradient_map-msg:GradientMap instead.")))

(cl:ensure-generic-function 'size_x-val :lambda-list '(m))
(cl:defmethod size_x-val ((m <GradientMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gradient_map-msg:size_x-val is deprecated.  Use gradient_map-msg:size_x instead.")
  (size_x m))

(cl:ensure-generic-function 'size_y-val :lambda-list '(m))
(cl:defmethod size_y-val ((m <GradientMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gradient_map-msg:size_y-val is deprecated.  Use gradient_map-msg:size_y instead.")
  (size_y m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <GradientMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gradient_map-msg:map-val is deprecated.  Use gradient_map-msg:map instead.")
  (map m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GradientMap>) ostream)
  "Serializes a message object of type '<GradientMap>"
  (cl:let* ((signed (cl:slot-value msg 'size_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'size_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'map))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GradientMap>) istream)
  "Deserializes a message object of type '<GradientMap>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GradientMap>)))
  "Returns string type for a message object of type '<GradientMap>"
  "gradient_map/GradientMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GradientMap)))
  "Returns string type for a message object of type 'GradientMap"
  "gradient_map/GradientMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GradientMap>)))
  "Returns md5sum for a message object of type '<GradientMap>"
  "a6366eb8d1d67d7bdd566f6df18f355a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GradientMap)))
  "Returns md5sum for a message object of type 'GradientMap"
  "a6366eb8d1d67d7bdd566f6df18f355a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GradientMap>)))
  "Returns full string definition for message of type '<GradientMap>"
  (cl:format cl:nil "int32 size_x~%int32 size_y~%float64[] map~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GradientMap)))
  "Returns full string definition for message of type 'GradientMap"
  (cl:format cl:nil "int32 size_x~%int32 size_y~%float64[] map~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GradientMap>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GradientMap>))
  "Converts a ROS message object to a list"
  (cl:list 'GradientMap
    (cl:cons ':size_x (size_x msg))
    (cl:cons ':size_y (size_y msg))
    (cl:cons ':map (map msg))
))
