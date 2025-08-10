
(cl:in-package :asdf)

(defsystem "transport-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TransportRequest" :depends-on ("_package_TransportRequest"))
    (:file "_package_TransportRequest" :depends-on ("_package"))
  ))