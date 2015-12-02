
(in-package :gps-search)

(defparameter *ops* nil)

(defun action-p (x)
  (or (equal x '(start)) (executing-p x)))

(defun executing-p (x)
  (starts-with x 'executing))

(defun gps-search (start goal &optional (beam-width 10) (blocks '(a b c)))
  (find-all-if  #'action-p
		(beam-search (cons '(start) start)
			     #'(lambda (state) (subsetp goal state :test #'equal))
			     #'gps-successors
			     #'(lambda (state)
				 (+ (count-if #'action-p state)
				    (count-if #'(lambda (con)
						  (not (member-equal con state)))
					      goal)))
			     beam-width)))

(defun gps-successors (state &optional (blocks '(a b c)))
  (mapcar #'(lambda (op)
	      (append
	       (remove-if #'(lambda (x)
			      (member-equal x (op-del-list op)))
			  state)
	       (op-add-list op)))
	  (applicable-ops state)))

(defun applicable-ops (state &optional (blocks '(a b c)))
  (find-all-if #'(lambda (op)
		   (subsetp (op-preconds op) state :test #'equal))
	       (make-block-ops blocks)))

(defun make-block-ops (blocks)
  (let ((ops nil))
    (dolist (a blocks)
      (dolist (b blocks)
        (unless (equal a b)
          (dolist (c blocks)
            (unless (or (equal c a) (equal c b))
              (push (move-op a b c) ops)))
          (push (move-op a 'table b) ops)
          (push (move-op a b 'table) ops))))
    ops))

(defun move-op (a b c)
  (op `(move ,a from ,b to ,c)
      :preconds `((space on ,a) (space on ,c) (,a on ,b))
      :add-list (move-ons a b c)
      :del-list (move-ons a c b)))

(defun move-ons (a b c)
  (if (eq b 'table)
      `((,a on ,c))
      `((,a on ,c) (space on ,b))))
