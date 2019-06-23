;; Auto-generated. Do not edit!


(when (boundp 'testmsg::Testmag)
  (if (not (find-package "TESTMSG"))
    (make-package "TESTMSG"))
  (shadow 'Testmag (find-package "TESTMSG")))
(unless (find-package "TESTMSG::TESTMAG")
  (make-package "TESTMSG::TESTMAG"))

(in-package "ROS")
;;//! \htmlinclude Testmag.msg.html


(defclass testmsg::Testmag
  :super ros::object
  :slots (_left _right ))

(defmethod testmsg::Testmag
  (:init
   (&key
    ((:left __left) 0.0)
    ((:right __right) 0.0)
    )
   (send-super :init)
   (setq _left (float __left))
   (setq _right (float __right))
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:serialization-length
   ()
   (+
    ;; float32 _left
    4
    ;; float32 _right
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left
       (sys::poke _left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right
       (sys::poke _right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left
     (setq _left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right
     (setq _right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get testmsg::Testmag :md5sum-) "3a927990ab5d5c3d628e2d52b8533e52")
(setf (get testmsg::Testmag :datatype-) "testmsg/Testmag")
(setf (get testmsg::Testmag :definition-)
      "float32 left
float32 right
")



(provide :testmsg/Testmag "3a927990ab5d5c3d628e2d52b8533e52")


