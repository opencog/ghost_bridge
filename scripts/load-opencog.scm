(use-modules (opencog)
             (opencog cogserver)
             (opencog logger)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog openpsi)
             (opencog attention)
             (opencog eva-behavior)
             (opencog ghost)
             (opencog ghost procedures)
             (opencog exec))

(define start-ecan-agents
  (string-append "agents-start "
    "opencog::AFImportanceDiffusionAgent "
    "opencog::WAImportanceDiffusionAgent "
    "opencog::WARentCollectionAgent "
    "opencog::AFRentCollectionAgent"))

(load "load-actions.scm")

(set-relex-server-host)
(start-cogserver "opencog.conf")
(cog-logger-set-stdout! #f)

; load and start the ghost-bridge-action-node
(use-modules (opencog ghost-bridge-action-node))
(start-ghost-bridge-action-node)

(define (ghost-set-ecan-filter . add-types)
  (apply ecan-set-spreading-filter
    (filter (lambda (x) (not (member x add-types))) (cog-get-types)))
)

(define (ec)
  (ghost-set-ecan-filter
    'ImplicationLink
    'WordNode
    'ConceptNode
    'PredicateNode
    'EvaluationLink
    'GroundedPredicateNode)
)

; Handle command-line arguments
(when (and (equal? 2 (length (command-line)))
           (equal? "enable-ecan" (cadr (command-line))))
  ; apply ECAN filter
  (ec)

  ; start the ECAN agents
  (system (string-append "echo \"" start-ecan-agents
    "\" | nc localhost 17001"))
)

; start recording perception inputs after a delay of few seconds.
(sleep 5)
(perception-start!)

; Start the ghost loop
(ghost-run)
