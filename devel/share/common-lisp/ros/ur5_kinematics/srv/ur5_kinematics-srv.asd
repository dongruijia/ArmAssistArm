
(cl:in-package :asdf)

(defsystem "ur5_kinematics-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IkService" :depends-on ("_package_IkService"))
    (:file "_package_IkService" :depends-on ("_package"))
  ))