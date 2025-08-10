;; Auto-generated. Do not edit!


(when (boundp 'robot::TransportRequest)
  (if (not (find-package "ROBOT"))
    (make-package "ROBOT"))
  (shadow 'TransportRequest (find-package "ROBOT")))
(unless (find-package "ROBOT::TRANSPORTREQUEST")
  (make-package "ROBOT::TRANSPORTREQUEST"))
(unless (find-package "ROBOT::TRANSPORTREQUESTREQUEST")
  (make-package "ROBOT::TRANSPORTREQUESTREQUEST"))
(unless (find-package "ROBOT::TRANSPORTREQUESTRESPONSE")
  (make-package "ROBOT::TRANSPORTREQUESTRESPONSE"))

(in-package "ROS")





(defclass robot::TransportRequestRequest
  :super ros::object
  :slots (_target ))

(defmethod robot::TransportRequestRequest
  (:init
   (&key
    ((:target __target) "")
    )
   (send-super :init)
   (setq _target (string __target))
   self)
  (:target
   (&optional __target)
   (if __target (setq _target __target)) _target)
  (:serialization-length
   ()
   (+
    ;; string _target
    4 (length _target)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _target
       (write-long (length _target) s) (princ _target s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _target
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _target (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot::TransportRequestResponse
  :super ros::object
  :slots (_success _message ))

(defmethod robot::TransportRequestResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot::TransportRequest
  :super ros::object
  :slots ())

(setf (get robot::TransportRequest :md5sum-) "e41a7a3cc3ea192648ce06af33a5d757")
(setf (get robot::TransportRequest :datatype-) "robot/TransportRequest")
(setf (get robot::TransportRequest :request) robot::TransportRequestRequest)
(setf (get robot::TransportRequest :response) robot::TransportRequestResponse)

(defmethod robot::TransportRequestRequest
  (:response () (instance robot::TransportRequestResponse :init)))

(setf (get robot::TransportRequestRequest :md5sum-) "e41a7a3cc3ea192648ce06af33a5d757")
(setf (get robot::TransportRequestRequest :datatype-) "robot/TransportRequestRequest")
(setf (get robot::TransportRequestRequest :definition-)
      "string target  # 请求参数：目标标记（A/B/C/D/CAMP/WAIT）
---
bool success   # 响应结果：是否成功
string message # 响应消息：状态描述


")

(setf (get robot::TransportRequestResponse :md5sum-) "e41a7a3cc3ea192648ce06af33a5d757")
(setf (get robot::TransportRequestResponse :datatype-) "robot/TransportRequestResponse")
(setf (get robot::TransportRequestResponse :definition-)
      "string target  # 请求参数：目标标记（A/B/C/D/CAMP/WAIT）
---
bool success   # 响应结果：是否成功
string message # 响应消息：状态描述


")



(provide :robot/TransportRequest "e41a7a3cc3ea192648ce06af33a5d757")


