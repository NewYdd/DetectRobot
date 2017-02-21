
(cl:in-package :asdf)

(defsystem "movement-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "wheelVelocity" :depends-on ("_package_wheelVelocity"))
    (:file "_package_wheelVelocity" :depends-on ("_package"))
  ))