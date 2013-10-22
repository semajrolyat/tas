
(cl:in-package :asdf)

(defsystem "pendulum-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rpc_command" :depends-on ("_package_rpc_command"))
    (:file "_package_rpc_command" :depends-on ("_package"))
    (:file "state" :depends-on ("_package_state"))
    (:file "_package_state" :depends-on ("_package"))
  ))