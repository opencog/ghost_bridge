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

; Start the ghost loop
(ghost-run)
