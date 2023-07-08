;; Auto-generated. Do not edit!


(when (boundp 'drone_test::code)
  (if (not (find-package "DRONE_TEST"))
    (make-package "DRONE_TEST"))
  (shadow 'code (find-package "DRONE_TEST")))
(unless (find-package "DRONE_TEST::CODE")
  (make-package "DRONE_TEST::CODE"))

(in-package "ROS")
;;//! \htmlinclude code.msg.html


(defclass drone_test::code
  :super ros::object
  :slots (_code ))

(defmethod drone_test::code
  (:init
   (&key
    ((:code __code) 0)
    )
   (send-super :init)
   (setq _code (round __code))
   self)
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:serialization-length
   ()
   (+
    ;; int8 _code
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _code
       (write-byte _code s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _code
     (setq _code (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _code 127) (setq _code (- _code 256)))
   ;;
   self)
  )

(setf (get drone_test::code :md5sum-) "95cfa23476470f3d4705c11337b96909")
(setf (get drone_test::code :datatype-) "drone_test/code")
(setf (get drone_test::code :definition-)
      "int8 code
")



(provide :drone_test/code "95cfa23476470f3d4705c11337b96909")


