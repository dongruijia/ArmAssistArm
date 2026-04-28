;; Auto-generated. Do not edit!


(when (boundp 'ur5_kinematics::IkService)
  (if (not (find-package "UR5_KINEMATICS"))
    (make-package "UR5_KINEMATICS"))
  (shadow 'IkService (find-package "UR5_KINEMATICS")))
(unless (find-package "UR5_KINEMATICS::IKSERVICE")
  (make-package "UR5_KINEMATICS::IKSERVICE"))
(unless (find-package "UR5_KINEMATICS::IKSERVICEREQUEST")
  (make-package "UR5_KINEMATICS::IKSERVICEREQUEST"))
(unless (find-package "UR5_KINEMATICS::IKSERVICERESPONSE")
  (make-package "UR5_KINEMATICS::IKSERVICERESPONSE"))

(in-package "ROS")





(defclass ur5_kinematics::IkServiceRequest
  :super ros::object
  :slots (_target_x _target_y _target_z _roll _pitch _yaw _joint_prev ))

(defmethod ur5_kinematics::IkServiceRequest
  (:init
   (&key
    ((:target_x __target_x) 0.0)
    ((:target_y __target_y) 0.0)
    ((:target_z __target_z) 0.0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:joint_prev __joint_prev) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _target_x (float __target_x))
   (setq _target_y (float __target_y))
   (setq _target_z (float __target_z))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _joint_prev __joint_prev)
   self)
  (:target_x
   (&optional __target_x)
   (if __target_x (setq _target_x __target_x)) _target_x)
  (:target_y
   (&optional __target_y)
   (if __target_y (setq _target_y __target_y)) _target_y)
  (:target_z
   (&optional __target_z)
   (if __target_z (setq _target_z __target_z)) _target_z)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:joint_prev
   (&optional __joint_prev)
   (if __joint_prev (setq _joint_prev __joint_prev)) _joint_prev)
  (:serialization-length
   ()
   (+
    ;; float64 _target_x
    8
    ;; float64 _target_y
    8
    ;; float64 _target_z
    8
    ;; float64 _roll
    8
    ;; float64 _pitch
    8
    ;; float64 _yaw
    8
    ;; float64[] _joint_prev
    (* 8    (length _joint_prev)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _target_x
       (sys::poke _target_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_y
       (sys::poke _target_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _target_z
       (sys::poke _target_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _joint_prev
     (write-long (length _joint_prev) s)
     (dotimes (i (length _joint_prev))
       (sys::poke (elt _joint_prev i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _target_x
     (setq _target_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_y
     (setq _target_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _target_z
     (setq _target_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _roll
     (setq _roll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pitch
     (setq _pitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _joint_prev
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_prev (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_prev i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass ur5_kinematics::IkServiceResponse
  :super ros::object
  :slots (_success _error_code _max_angle_error_deg _joint_positions ))

(defmethod ur5_kinematics::IkServiceResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:error_code __error_code) 0)
    ((:max_angle_error_deg __max_angle_error_deg) 0.0)
    ((:joint_positions __joint_positions) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _success __success)
   (setq _error_code (round __error_code))
   (setq _max_angle_error_deg (float __max_angle_error_deg))
   (setq _joint_positions __joint_positions)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:max_angle_error_deg
   (&optional __max_angle_error_deg)
   (if __max_angle_error_deg (setq _max_angle_error_deg __max_angle_error_deg)) _max_angle_error_deg)
  (:joint_positions
   (&optional __joint_positions)
   (if __joint_positions (setq _joint_positions __joint_positions)) _joint_positions)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _error_code
    4
    ;; float64 _max_angle_error_deg
    8
    ;; float64[] _joint_positions
    (* 8    (length _joint_positions)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _error_code
       (write-long _error_code s)
     ;; float64 _max_angle_error_deg
       (sys::poke _max_angle_error_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _joint_positions
     (write-long (length _joint_positions) s)
     (dotimes (i (length _joint_positions))
       (sys::poke (elt _joint_positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _error_code
     (setq _error_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _max_angle_error_deg
     (setq _max_angle_error_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _joint_positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass ur5_kinematics::IkService
  :super ros::object
  :slots ())

(setf (get ur5_kinematics::IkService :md5sum-) "3ec0d754206886798b3d26a7d01862a6")
(setf (get ur5_kinematics::IkService :datatype-) "ur5_kinematics/IkService")
(setf (get ur5_kinematics::IkService :request) ur5_kinematics::IkServiceRequest)
(setf (get ur5_kinematics::IkService :response) ur5_kinematics::IkServiceResponse)

(defmethod ur5_kinematics::IkServiceRequest
  (:response () (instance ur5_kinematics::IkServiceResponse :init)))

(setf (get ur5_kinematics::IkServiceRequest :md5sum-) "3ec0d754206886798b3d26a7d01862a6")
(setf (get ur5_kinematics::IkServiceRequest :datatype-) "ur5_kinematics/IkServiceRequest")
(setf (get ur5_kinematics::IkServiceRequest :definition-)
      "# 请求数据 Request
float64 target_x
float64 target_y
float64 target_z
float64 roll
float64 pitch
float64 yaw
float64[] joint_prev

---
# 响应数据 Response（必须加 --- 分隔！）
bool success
int32 error_code
float64 max_angle_error_deg
float64[] joint_positions
")

(setf (get ur5_kinematics::IkServiceResponse :md5sum-) "3ec0d754206886798b3d26a7d01862a6")
(setf (get ur5_kinematics::IkServiceResponse :datatype-) "ur5_kinematics/IkServiceResponse")
(setf (get ur5_kinematics::IkServiceResponse :definition-)
      "# 请求数据 Request
float64 target_x
float64 target_y
float64 target_z
float64 roll
float64 pitch
float64 yaw
float64[] joint_prev

---
# 响应数据 Response（必须加 --- 分隔！）
bool success
int32 error_code
float64 max_angle_error_deg
float64[] joint_positions
")



(provide :ur5_kinematics/IkService "3ec0d754206886798b3d26a7d01862a6")


