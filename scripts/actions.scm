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
; TODO: I think 'stop' might be better than 'cancel' for stopping each action
;

(define set-parameter "set-parameter")

(define gaze-at "gaze-at")
(define say "say")
(define say-cancel "say-cancel")

(define gaze-at-cancel "gaze-at-cancel")

(define blink "blink")
(define blink-cancel "blink-cancel")

(define saccade "saccade")
(define saccade-cancel "saccade-cancel")

(define emote "emote")
(define gesture "gesture")

(define soma "soma")
(define soma-cancel "soma-cancel")

; -------------------------------------------------------------
; Vary the values of parameters of different components
;
; Example usage:
;   (cog-execute!
;     (Put (DefinedSchema "set-parameter")
;       (List (Concept "speech") (Concept "volume") (Concept "2"))))


(delete-definition set-parameter)
(DefineLink
 (DefinedSchema set-parameter)
 (LambdaLink
  (VariableList
   (Variable "$component")
   (Variable "$parameter")
   (Variable "$value"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:set_parameter")
    (ListLink
     (Variable "$component")
     (Variable "$parameter")
     (Variable "$value")))
  )))

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


; -------------------------------------------------------------
; Cancel the current utterance being spoken
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "say-cancel") (List)))
;

(delete-definition say-cancel)
(DefineLink
 (DefinedSchema say-cancel)
 (LambdaLink
  (SequentialAndLink
   (EvaluationLink
    (GroundedPredicate "py:say_cancel")
    (List))
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
; Cancel the current gaze-at action
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "gaze-at-cancel") (List)))
;

(delete-definition gaze-at-cancel)
(DefineLink
 (DefinedSchema gaze-at-cancel)
 (LambdaLink
  (SequentialAndLink
   (EvaluationLink
    (GroundedPredicate "py:gaze_at_cancel")
    (List))
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
; Cancel the current blink cycle
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "blink-cancel") (List)))
;

(delete-definition blink-cancel)
(DefineLink
 (DefinedSchema blink-cancel)
 (LambdaLink
  (SequentialAndLink
   (EvaluationLink
    (GroundedPredicate "py:blink_cancel")
    (List))
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
; Cancel the current saccade cycle
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "saccade-cancel") (List)))
;

(delete-definition saccade-cancel)
(DefineLink
 (DefinedSchema saccade-cancel)
 (LambdaLink
  (SequentialAndLink
   (EvaluationLink
    (GroundedPredicate "py:saccade_cancel")
    (List))
  )))


; -------------------------------------------------------------
; Request a display of an emotion (smile, frown, etc.)
; The expression type should be one of the supported blender animations.
;
; blend the emotion with other emotions that also have blend=True. If an emotion is sent with blend=False, then it will
; overwrite all previously sent and active blendable emotions.
;
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "emote") (List (Concept "happy") (Number 0.6) (Number 2.0) (Concept "False"))))
;

(delete-definition emote)
(DefineLink
 (DefinedSchema emote)
 (LambdaLink
  (VariableList
   (Variable "$name")
   (Variable "$magnitude")
   (Variable "$duration")
   (Variable "$blend"))
  (SequentialAndLink
   (EvaluationLink (GroundedPredicate "py:emote")
    (ListLink
     (Variable "$name")
     (Variable "$magnitude")
     (Variable "$duration")
     (Variable "$blend")))
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


; -------------------------------------------------------------
; Cancel the current soma cycle
;
; Example usage:
;   (cog-execute! (Put (DefinedSchema "soma-cancel") (List (Concept "normal"))))
;

(delete-definition soma-cancel)
(DefineLink
 (DefinedSchema soma-cancel)
 (LambdaLink
  (VariableList
   (Variable "$name"))
  (SequentialAndLink
   (EvaluationLink
    (GroundedPredicate "py:soma_cancel")
    (ListLink
     (Variable "$name")))
  )))


*unspecified* ; Make the load be silent
