
(cl:in-package :asdf)

(defsystem "pendulum-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "synch" :depends-on ("_package_synch"))
    (:file "_package_synch" :depends-on ("_package"))
    (:file "command" :depends-on ("_package_command"))
    (:file "_package_command" :depends-on ("_package"))
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
  ))