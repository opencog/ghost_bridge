;
; actions.scm
;
; Implement the Ghost action API for ROS
;

; -------------------------------------------------------------
; Delete the current definition, if any.
;

(define (delete-definition STR)
 (define dfn
  (cog-get-link 'DefineLink 'DefinedPredicateNode
   (DefinedPredicate STR)))

 (if (not (null? dfn)) (cog-delete (car dfn)) #f))


; -------------------------------------------------------------
; Action name definitions
;

(define say "say")
(define gaze-at "gaze-at")
(define face-toward "face-toward")
(define blink "blink")
(define saccade "saccade")
(define emote "emote")
(define gesture "gesture")
(define soma "soma")

; -------------------------------------------------------------
; Say something.
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "say") (Node "this is a test")))
;

(delete-definition say)
(DefineLink
 (DefinedPredicate say)
 (LambdaLink (Variable "text")
  (Evaluation
   (GroundedPredicate "py:say")
   (List (Variable "text")))
 ))


;---------------------------------------------------------------
; Request robot to point its eyes at a specific point
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "gaze-at") (List (Number 1.3) (Number 1.2) (Number 1.1) (Number 0.5))))
;

(delete-definition gaze-at)
(DefineLink
 (DefinedPredicate gaze-at)
 (LambdaLink
  (VariableList
   (Variable "$x")
   (Variable "$y")
   (Variable "$z")
   (Variable "$speed"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:gaze_at")
    (ListLink
     (Variable "$x")
     (Variable "$y")
     (Variable "$z")
     (Variable "$speed")))
  )))


;---------------------------------------------------------------
; Request robot to turn its face toward a specific point
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "face-toward") (List (Number 1.3) (Number 1.2) (Number 1.1) (Number 0.5))))
;

(delete-definition face-toward)
(DefineLink
 (DefinedPredicate face-toward)
 (LambdaLink
  (VariableList
   (Variable "$x")
   (Variable "$y")
   (Variable "$z")
   (Variable "$speed"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:face_toward")
    (ListLink
     (Variable "$x")
     (Variable "$y")
     (Variable "$z")
     (Variable "$speed")))
  )))


; -------------------------------------------------------------
; Request a display of a facial expression (smile, frown, etc.)
; The expression type should be one of the supported blender animations.
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "blink") (List (Number 5.5) (Number 1.5))))
;

(delete-definition blink)
(DefineLink
 (DefinedPredicate blink)
 (LambdaLink
  (VariableList
   (Variable "$mean")
   (Variable "$variation"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:blink")
    (ListLink
     (Variable "$mean")
     (Variable "$variation")))
  )))


; -------------------------------------------------------------
; Request a display of a facial expression (smile, frown, etc.)
; The expression type should be one of the supported blender animations.
;
; Example usage:
;    (cog-evaluate! (Put (DefinedPredicate "saccade")
;         (List
;           (Number 0.8)
;           (Number 0.3)
;           (Number 0.3)
;           (Number 15.0)
;           (Number 100.0)
;           (Number 90.0)
;           (Number 27.0)
;           (Number 0.8)
;           (Number 0.2)
;         )
;    ))

(delete-definition saccade)
(DefineLink
 (DefinedPredicate saccade)
 (LambdaLink
  (VariableList
   (Variable "$mean")
   (Variable "$variation")
   (Variable "$paint_scale")
   (Variable "$eye_size")
   (Variable "$eye_distance")
   (Variable "$mouth_width")
   (Variable "$mouth_height")
   (Variable "$weight_eyes")
   (Variable "$weight_mouth"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:saccade")
    (ListLink
     (Variable "$mean")
     (Variable "$variation")
     (Variable "$paint_scale")
     (Variable "$eye_size")
     (Variable "$eye_distance")
     (Variable "$mouth_width")
     (Variable "$mouth_height")
     (Variable "$weight_eyes")
     (Variable "$weight_mouth")))
  )))


; -------------------------------------------------------------
; Request a display of an emotion (smile, frown, etc.)
; The expression type should be one of the supported blender animations.
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "emote") (List (Concept "happy") (Number 0.6) (Number 2.0))))
;

(delete-definition emote)
(DefineLink
 (DefinedPredicate emote)
 (LambdaLink
  (VariableList
   (Variable "$name")
   (Variable "$magnitude")
   (Variable "$duration"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:emote")
    (ListLink
     (Variable "$name")
     (Variable "$magnitude")
     (Variable "$duration")))
  )))


; -------------------------------------------------------------
; Request a display of a facial gesture (blink, nod, etc.)
; The expression name should be one of the supported blender animations.
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "gesture") (List (Concept "blink") (Number 3) (Number 0.8) (Number 1))))
;

(delete-definition gesture)
(DefineLink
 (DefinedPredicate gesture)
 (LambdaLink
  (VariableList
   (Variable "$name")
   (Variable "$speed")
   (Variable "$magnitude")
   (Variable "$repeat"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:gesture")
    (ListLink
     (Variable "$name")
     (Variable "$speed")
     (Variable "$magnitude")
     (Variable "$repeat")))
  )))


; -------------------------------------------------------------
; Request a display of a soma state
;
; Example usage:
;   (cog-evaluate! (Put (DefinedPredicate "soma") (List (Concept "normal") (Number 0.1) (Number 1) (Number 3))))
;

(delete-definition soma)
(DefineLink
 (DefinedPredicate soma)
 (LambdaLink
  (VariableList
   (Variable "$name")
   (Variable "$magnitude")
   (Variable "$rate")
   (Variable "$ease_in"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:soma")
    (ListLink
     (Variable "$name")
     (Variable "$magnitude")
     (Variable "$rate")
     (Variable "$ease_in")))
  )))

*unspecified* ; Make the load be silent