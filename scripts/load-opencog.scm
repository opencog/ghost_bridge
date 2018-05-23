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


(load "load-actions.scm")

(ecan-based-ghost-rules #t)
(set-relex-server-host)

(start-cogserver "opencog.conf")
(cog-logger-set-stdout! #f)

; load and start the ghost-bridge-action-node
(use-modules (opencog ghost-bridge-action-node))
(start-ghost-bridge-action-node)

(define SPREADING_FILTER (ConceptNode "SPREADING_FILTER"))

(define-public (ecan-set-spreading-filter . type-symbols)
"
  ecan-set-spreading-filter TYPE-SYMBOLS
  Set ecan to filter atoms of TYPE-SYMOBLS.
"
  (if (not (null? type-symbols))
    (StateLink
      SPREADING_FILTER
      (MemberLink
        (map (lambda (x) (TypeNode (symbol->string x))) type-symbols))))

  SPREADING_FILTER
)

(define (ghost-set-ecan-filter . add-types)
  (apply ecan-set-spreading-filter
    (filter (lambda (x) (not (member x add-types))) (cog-get-types)))
)

(define (ec)
  (ghost-set-ecan-filter
    'ImplicationLink 'WordNode 'ConceptNode 'PredicateNode 'EvaluationLink)
)

(ec)

; Start the ghost loop
(ghost-run)
