;; Auto-generated. Do not edit!


(when (boundp 'drone_test::Y_Serial_port)
  (if (not (find-package "DRONE_TEST"))
    (make-package "DRONE_TEST"))
  (shadow 'Y_Serial_port (find-package "DRONE_TEST")))
(unless (find-package "DRONE_TEST::Y_SERIAL_PORT")
  (make-package "DRONE_TEST::Y_SERIAL_PORT"))

(in-package "ROS")
;;//! \htmlinclude Y_Serial_port.msg.html


(defclass drone_test::Y_Serial_port
  :super ros::object
  :slots (_mode _tar1 _tar2 _if_takeoff ))

(defmethod drone_test::Y_Serial_port
  (:init
   (&key
    ((:mode __mode) 0)
    ((:tar1 __tar1) 0)
    ((:tar2 __tar2) 0)
    ((:if_takeoff __if_takeoff) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _tar1 (round __tar1))
   (setq _tar2 (round __tar2))
   (setq _if_takeoff (round __if_takeoff))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:tar1
   (&optional __tar1)
   (if __tar1 (setq _tar1 __tar1)) _tar1)
  (:tar2
   (&optional __tar2)
   (if __tar2 (setq _tar2 __tar2)) _tar2)
  (:if_takeoff
   (&optional __if_takeoff)
   (if __if_takeoff (setq _if_takeoff __if_takeoff)) _if_takeoff)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ;; int8 _tar1
    1
    ;; int8 _tar2
    1
    ;; int8 _if_takeoff
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;; int8 _tar1
       (write-byte _tar1 s)
     ;; int8 _tar2
       (write-byte _tar2 s)
     ;; int8 _if_takeoff
       (write-byte _if_takeoff s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; int8 _tar1
     (setq _tar1 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _tar1 127) (setq _tar1 (- _tar1 256)))
   ;; int8 _tar2
     (setq _tar2 (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _tar2 127) (setq _tar2 (- _tar2 256)))
   ;; int8 _if_takeoff
     (setq _if_takeoff (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _if_takeoff 127) (setq _if_takeoff (- _if_takeoff 256)))
   ;;
   self)
  )

(setf (get drone_test::Y_Serial_port :md5sum-) "ae4cda509b27984341a3ec103b7367d7")
(setf (get drone_test::Y_Serial_port :datatype-) "drone_test/Y_Serial_port")
(setf (get drone_test::Y_Serial_port :definition-)
      "int8 mode
int8 tar1
int8 tar2
int8 if_takeoff 


")



(provide :drone_test/Y_Serial_port "ae4cda509b27984341a3ec103b7367d7")


