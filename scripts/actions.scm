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
  (cog-get-link 'DefineLink 'DefinedSchemaNode
   (DefinedSchema STR)))

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
;   (cog-execute! (Put (DefinedSchema "say") (List (Concept "this is a test") (Concept ""))))
;   (cog-execute! (Put (DefinedSchema "say") (List (Concept "") (Concept "chatscript"))))
;

(delete-definition say)
(DefineLink
 (DefinedSchema say)
 (LambdaLink
  (VariableList
   (Variable "$text")
   (Variable "$fallback_id"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:say")
    (ListLink
     (Variable "$text")
     (Variable "$fallback_id")))
  )))


;---------------------------------------------------------------
; Request the robot to point its face and eyes at a specific face_id
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "gaze-at") (List (Concept "aef7dfsd89f8dsf9dsf97dsf") (Number 0.5))))
;

(delete-definition gaze-at)
(DefineLink
 (DefinedSchema gaze-at)
 (LambdaLink
  (VariableList
   (Variable "$face_id")
   (Variable "$speed"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:gaze_at")
    (ListLink
     (Variable "$face_id")
     (Variable "$speed")))
  )))

; -------------------------------------------------------------
; Request a display of a facial expression (smile, frown, etc.)
; The expression type should be one of the supported blender animations.
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "blink") (List (Number 5.5) (Number 1.5))))
;

(delete-definition blink)
(DefineLink
 (DefinedSchema blink)
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
;    (cog-execute! (Put (DefinedSchema "saccade")
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
 (DefinedSchema saccade)
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
;   (cog-execute! (Put (DefinedSchema "emote") (List (Concept "happy") (Number 0.6) (Number 2.0))))
;

(delete-definition emote)
(DefineLink
 (DefinedSchema emote)
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
;   (cog-execute! (Put (DefinedSchema "gesture") (List (Concept "025_noddingHappy") (Number 3) (Number 0.8) (Number 1))))
;

(delete-definition gesture)
(DefineLink
 (DefinedSchema gesture)
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
;   (cog-execute! (Put (DefinedSchema "soma") (List (Concept "normal") (Number 0.1) (Number 1) (Number 3))))
;   (cog-execute! (Put (DefinedSchema "soma") (List (Concept "sleep") (Number 1) (Number 1) (Number 3))))
;

(delete-definition soma)
(DefineLink
 (DefinedSchema soma)
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
