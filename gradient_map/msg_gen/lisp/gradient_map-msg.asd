
(cl:in-package :asdf)

(defsystem "gradient_map-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GradientMap" :depends-on ("_package_GradientMap"))
    (:file "_package_GradientMap" :depends-on ("_package"))
  ))