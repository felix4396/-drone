
(cl:in-package :asdf)

(defsystem "drone_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Y_Serial_port" :depends-on ("_package_Y_Serial_port"))
    (:file "_package_Y_Serial_port" :depends-on ("_package"))
    (:file "code" :depends-on ("_package_code"))
    (:file "_package_code" :depends-on ("_package"))
    (:file "detection" :depends-on ("_package_detection"))
    (:file "_package_detection" :depends-on ("_package"))
  ))