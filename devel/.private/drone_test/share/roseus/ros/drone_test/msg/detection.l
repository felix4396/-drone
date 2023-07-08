;; Auto-generated. Do not edit!


(when (boundp 'drone_test::detection)
  (if (not (find-package "DRONE_TEST"))
    (make-package "DRONE_TEST"))
  (shadow 'detection (find-package "DRONE_TEST")))
(unless (find-package "DRONE_TEST::DETECTION")
  (make-package "DRONE_TEST::DETECTION"))

(in-package "ROS")
;;//! \htmlinclude detection.msg.html


(defclass drone_test::detection
  :super ros::object
  :slots (_erro_x _erro_y _flag _start_opencv ))

(defmethod drone_test::detection
  (:init
   (&key
    ((:erro_x __erro_x) 0.0)
    ((:erro_y __erro_y) 0.0)
    ((:flag __flag) 0)
    ((:start_opencv __start_opencv) 0)
    )
   (send-super :init)
   (setq _erro_x (float __erro_x))
   (setq _erro_y (float __erro_y))
   (setq _flag (round __flag))
   (setq _start_opencv (round __start_opencv))
   self)
  (:erro_x
   (&optional __erro_x)
   (if __erro_x (setq _erro_x __erro_x)) _erro_x)
  (:erro_y
   (&optional __erro_y)
   (if __erro_y (setq _erro_y __erro_y)) _erro_y)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:start_opencv
   (&optional __start_opencv)
   (if __start_opencv (setq _start_opencv __start_opencv)) _start_opencv)
  (:serialization-length
   ()
   (+
    ;; float64 _erro_x
    8
    ;; float64 _erro_y
    8
    ;; int8 _flag
    1
    ;; int8 _start_opencv
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _erro_x
       (sys::poke _erro_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _erro_y
       (sys::poke _erro_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _flag
       (write-byte _flag s)
     ;; int8 _start_opencv
       (write-byte _start_opencv s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _erro_x
     (setq _erro_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _erro_y
     (setq _erro_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _flag
     (setq _flag (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _flag 127) (setq _flag (- _flag 256)))
   ;; int8 _start_opencv
     (setq _start_opencv (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _start_opencv 127) (setq _start_opencv (- _start_opencv 256)))
   ;;
   self)
  )

(setf (get drone_test::detection :md5sum-) "5758eab71db35db21ff4cedc29365b18")
(setf (get drone_test::detection :datatype-) "drone_test/detection")
(setf (get drone_test::detection :definition-)
      "float64 erro_x
float64 erro_y
int8 flag
int8 start_opencv
")



(provide :drone_test/detection "5758eab71db35db21ff4cedc29365b18")


