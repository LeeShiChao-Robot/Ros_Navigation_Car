
(cl:in-package :asdf)

(defsystem "testmsg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Testmag" :depends-on ("_package_Testmag"))
    (:file "_package_Testmag" :depends-on ("_package"))
  ))